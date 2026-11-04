#pragma once
// SimDriver: the physics-ahead loop and the render interpolation (plan sections 4.3, 4.4, 6.6, 8).
//
// Owns a Simulation, a SimClock and a Warp. Once per real frame the app calls advance(frameDt):
//   1. the clock moves the display time:  t_target += warp * frameDt  (clock.h);
//   2. the physics runs ahead until its latest body step ends at or after t_target (Simulation::advanceTo);
//   3. DisplayState is built AT EXACTLY t_target: bodies from the IAS15 dense output of the latest body step, shuttles
//      by cubic Hermite from the recorded samples of that step (exact at sample times, continuous in between);
//   4. the events of this frame are handed out.
// Because the physics grid never depends on the targets (simulation.h), the DisplayState at a given t_target is the
// same whatever frames, warps or thread counts led there (bit for bit, tested).
//
// Display time is where the viewer is, simulation time (sim().time()) is where the physics is: the latter is at or
// past the former, by up to one body step. Events therefore come in two lists: everything the physics produced this
// frame (drainedThisFrame) and the events whose time the display has reached (dueEvents). Effects (explosions, log
// lines, warp slow-down) want the second so that nothing is shown before it is visible.
//
// Bodies that merge: the absorbed body is still drawn (absorbed = true, from the dense output) until the merge time,
// then it disappears and the survivor takes the merged state. Bodies and shuttles present in only one of the two
// states (added after the step, killed in it) are drawn without interpolation (plan 4.3).
//
// One frame cannot do unbounded work: when the physics needs more than DriverOptions::physicsBudget_s of wall time
// to reach t_target, it stops at a body step boundary, the display time WAITS at the physics time (physicsBehind is
// true, so the UI can say so) and the next frame goes on from there. This changes how fast the simulated clock runs,
// never a trajectory.
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include "app/clock.h"
#include "app/warp.h"
#include "core/simulation.h"

namespace cosmos {
namespace app {

struct DriverOptions {
    double maxFrameDt = SimClock::kDefaultMaxFrameDt;   // s, real time (SimClock)
    // Wall-time cap of the physics per frame, s (25 ms leaves room for drawing at 144 Hz when the physics is the only
    // load). <= 0 means no cap (headless runs, tests).
    double physicsBudget_s = 0.025;
};

// A body at the display time. Properties are the ones in the store at that time (see the merge note above).
struct DisplayBody {
    BodyId id = 0;
    std::string name;
    BodyKind kind = BodyKind::Planet;
    double mu = 0.0;
    double radius = 0.0;
    double spinRate = 0.0;
    std::optional<Atmosphere> atmosphere;
    std::array<float, 3> colour = {1.0f, 1.0f, 1.0f};
    Vec3 r;                      // position at the display time, simulation frame, m
    Vec3 v;                      // velocity, m/s
    bool interpolated = false;   // from the dense output (false: the store state at sim().time())
    bool absorbed = false;       // merges into another body at a time the display has not reached yet
};

// A shuttle at the display time. Dead shuttles appear only until their contact time (dying = true).
struct DisplayShuttle {
    ShuttleId id = 0;
    std::size_t index = 0;       // index into sim().shuttles()
    Vec3 r;                      // m
    Vec3 v;                      // m/s
    double heading = 0.0;        // rad (extrapolated back from the end of its last step with the spin and torque)
    double spin = 0.0;           // rad/s
    double throttle = 0.0;       // 0..1, the held command
    bool interpolated = false;   // from the recorded steps (false: it has not stepped yet)
    bool dying = false;          // destroyed at a time the display has not reached yet
};

struct DisplayState {
    double t = 0.0;              // the display time this state belongs to
    std::vector<DisplayBody> bodies;
    std::vector<DisplayShuttle> shuttles;
};

// What the last frame did.
struct FrameInfo {
    double frameDt = 0.0;        // real seconds used (after the clamp)
    double warp = 1.0;           // warp used
    double simDelta = 0.0;       // how far the display time actually moved (less than warp * frameDt when the physics was behind)
    double target = 0.0;         // display time t_target after the frame
    double simTime = 0.0;        // physics time (end of the latest body step)
    double lagSimSeconds = 0.0;  // how far the physics fell short of the wanted t_target (0 unless behind)
    double physicsSeconds = 0.0; // wall time the physics took
    long bodySteps = 0;          // body steps taken
    bool physicsBehind = false;
    bool failed = false;         // the integrator failed; the simulation stays where it was
};

class SimDriver {
public:
    // Builds the Simulation (recordShuttlePaths is forced on: the interpolation needs every shuttle step of the latest
    // body step) and the first DisplayState at simOptions.startTime.
    SimDriver(const ResolvedScene& scene, const SceneSettings& settings, SimulationOptions simOptions = {},
              DriverOptions options = {});
    SimDriver(const SimDriver&) = delete;
    SimDriver& operator=(const SimDriver&) = delete;

    // One real frame at the current warp. Returns the frame report.
    const FrameInfo& advance(double frameDt);
    // Move the display time to t (if later than now) without the warp or the frame clamp: headless runs, seeking, tests.
    // The same physics budget applies.
    const FrameInfo& advanceTo(double t);

    // ---- Controls ---------------------------------------------------------------------------------------------
    Warp& warp() { return warp_; }
    const Warp& warp() const { return warp_; }
    SimClock& clock() { return clock_; }
    const SimClock& clock() const { return clock_; }
    void setPaused(bool p) { clock_.setPaused(p); }
    bool paused() const { return clock_.paused(); }

    // ---- Results ----------------------------------------------------------------------------------------------
    double targetTime() const { return clock_.target(); }
    double simTime() const { return sim_.time(); }
    bool physicsBehind() const { return info_.physicsBehind; }
    const FrameInfo& frameInfo() const { return info_; }
    // The state at the display time, rebuilt by every advance().
    const DisplayState& state() const { return state_; }
    // The state at any t (clamped into the latest body step, so only t in [bodyStepStart, bodyStepEnd] is meaningful;
    // before the first step it is the initial state). Used by tests and by tools that sample between frames.
    DisplayState stateAt(double t) const;
    // Events the physics produced during the last advance, sorted by (t, id); may lie after the display time.
    const std::vector<Event>& drainedThisFrame() const { return drained_; }
    // Events whose time the display has reached for the first time this frame (each event appears exactly once).
    const std::vector<Event>& dueEvents() const { return due_; }
    // Events produced by the physics that the display has not reached yet.
    std::size_t pendingEventCount() const { return pending_.size(); }

    // The simulation, for edits while paused (controllers, adding a shuttle) and for diagnostics. After an edit call
    // advance(0) or advanceTo(targetTime()) to refresh state().
    Simulation& sim() { return sim_; }
    const Simulation& sim() const { return sim_; }

private:
    struct PendingMerge {
        BodyId survivor = 0;
        BodyId absorbed = 0;
        double t = 0.0;
        DisplayBody absorbedBefore;    // properties just before the merge
        DisplayBody survivorBefore;
        bool haveSurvivorBefore = false;
    };

    const FrameInfo& runFrame(double wantedTarget, double previousTarget);
    void runPhysics(double target);
    void collectEvents();
    void snapshotBodies(std::vector<DisplayBody>& out) const;
    void buildState(double t, DisplayState& out) const;
    const PendingMerge* mergeOf(BodyId survivor) const;

    Simulation sim_;
    SimClock clock_;
    Warp warp_;
    DriverOptions options_;

    FrameInfo info_;
    DisplayState state_;
    std::vector<DisplayBody> before_;       // store properties before this frame's physics (for merges)
    std::vector<Event> drained_, pending_, due_;
    std::vector<PendingMerge> merges_;
};

} // namespace app
} // namespace cosmos
