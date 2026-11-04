#pragma once
// Run control (plan sections 3 Q17/Q19, 10.3): Start / Pause / Stop / Save current as scene, and the glue between the
// editor model and the SimDriver. No OpenGL, no ImGui.
//
// RunControl owns the EditorModel, the snapshot taken at Start and the SimDriver (and with it the Simulation), and keeps
// the three together:
//
//   state    editor            driver / simulation
//   ------   ----------------  ----------------------------------------------------------------------------------
//   Stopped  editable          built from the editor's scene at its epoch, frozen (never advanced)
//   Running  LOCKED            advancing (advance() steps it); the editor still holds the scene from before
//   Paused   editable          frozen at the moment of the pause; the editor holds a CAPTURE of it (see below)
//
// Transitions
//   start()  Stopped -> Running. Takes the SNAPSHOT (a copy of the editor's scene) and builds a fresh driver from it, so the
//            run always starts from exactly the scene the user sees (same bits as any other start from that scene).
//            Paused -> Running: resumes. With no edits since the pause the very same driver continues, bit for bit as if
//            it had never been paused; with edits the simulation is rebuilt first (applyEdits below).
//   pause()  Running -> Paused. Freezes the clock and CAPTURES the simulation (captureScene) into the editor: the editor now
//            lists the bodies and shuttles as they are (merged bodies as one, destroyed shuttles gone, fuel left, positions
//            and velocities as explicit states) and is unlocked. The editor resolves this scene with Recentre::Never, so
//            editing never moves the bodies you did not touch.
//   stop()   Running or Paused -> Stopped. Restores the snapshot taken at Start: the editor gets the snapshot back
//            (everything edited or captured since is discarded), the driver is rebuilt from it, and the state is
//            bit-identical to the one before Start. Name and ID counters are NOT rolled back, so a body added and lost in a
//            run does not give its number to the next one.
//   saveCurrent / captureCurrent: "Save current as scene": keeps the state of the run as a scene (below).
//
// Editing while Paused (plan Q19): the setters of the editor change the captured scene. applyEdits() then REBUILDS the
// simulation from the edited scene: epoch = the simulation time of the pause (sim().time(), the end of the latest body
// step), clock at that epoch, warp unchanged, a fresh IAS15 (new step grid) and fresh shuttle integrators. So after an edit
// the trajectories are no longer bit-identical to what an unedited continuation would have produced (the physics is the
// same, the step sequence restarts), and a shuttle restarts with its heading along its velocity relative to its parent, spin 0
// and its controller state fresh, because a Scene has no heading, spin or throttle. Without an edit nothing is rebuilt.
// applyEdits() is what the panel calls after a successful Enter; start() and advance() also apply pending edits.
//
// Captured scene ("Save current"): epoch_s = the simulation time, every body in the store (a merged body is one body with
// the combined mass and the merge-time kind; the absorbed one is gone) with its position and velocity AS EXPLICIT STATES,
// spin period, atmosphere, colour and flags; every ALIVE shuttle with its current fuel; settings (infinite fuel, controller
// tick). Resolving it (it has no orbits, so nothing is recentred) gives back the same positions, velocities, masses and radii
// bit for bit; the spin rate returns to within a few ulp (period <-> rate conversion). IDs and names are kept.
//
// Pointers: driver() is a different object after start from Stopped, stop, loadScene and applyEdits that rebuilt. generation()
// changes then, so the app re-reads whatever it cached (body lookups, camera follow target) when it differs from last frame.
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "app/editor_model.h"
#include "app/sim_driver.h"
#include "core/scene.h"
#include "core/simulation.h"

namespace cosmos {
namespace app {

enum class RunState { Stopped, Running, Paused };
const char* runStateName(RunState s);

struct RunOptions {
    SimulationOptions sim;   // threads, integrator and shuttle tolerances; startTime is replaced by the scene epoch
    DriverOptions driver;
};

// The current state of a simulation as a Scene with explicit states (see the file comment). `name` becomes the scene name.
Scene captureScene(const Simulation& sim, const std::string& name, const std::string& units = "SI");

// Resolves the scene and builds a driver at its epoch (sim.startTime is set to scene.epoch_s). Null with a message in
// `error` when the scene is invalid. `warnings` (optional) gets the resolver's and the simulation's warnings.
std::unique_ptr<SimDriver> buildDriver(const Scene& scene, const ResolveOptions& resolve, SimulationOptions sim,
                                       const DriverOptions& driver, std::string& error,
                                       std::vector<std::string>* warnings = nullptr);

class RunControl {
public:
    // Starts Stopped with the empty sandbox.
    explicit RunControl(const RunOptions& options = {});
    RunControl(const RunControl&) = delete;
    RunControl& operator=(const RunControl&) = delete;

    // ---- Parts ------------------------------------------------------------------------------------------------
    EditorModel& editor() { return editor_; }
    const EditorModel& editor() const { return editor_; }
    // Always valid. A new object after a rebuild; see generation().
    SimDriver& driver() { return *driver_; }
    const SimDriver& driver() const { return *driver_; }
    std::uint64_t generation() const { return generation_; }
    RunOptions& options() { return options_; }   // takes effect at the next rebuild

    RunState state() const { return state_; }
    bool running() const { return state_ == RunState::Running; }
    bool canEdit() const { return state_ != RunState::Running; }
    // The scene Stop returns to; null before the first Start (and after loadScene).
    const Scene* snapshot() const { return haveSnapshot_ ? &snapshot_ : nullptr; }
    // Message of the last operation that failed or of an integrator failure ("" if none)
    const std::string& lastError() const { return lastError_; }
    // Resolver and simulation warnings of the last build
    const std::vector<std::string>& buildWarnings() const { return buildWarnings_; }

    // ---- Documents --------------------------------------------------------------------------------------------
    // Replaces the document (preset or file): stops any run, discards the snapshot, resets the name and ID counters from the
    // scene and rebuilds the driver. The warp setting stays. False with a message (nothing changed) if the scene cannot be
    // resolved.
    bool loadScene(Scene scene, std::string& error);

    // ---- Run control (EditResult.error says why not) -----------------------------------------------------------------
    EditResult start();
    EditResult pause();
    EditResult stop();

    // ---- Edits ------------------------------------------------------------------------------------------------
    // The editor's scene differs from the one the driver was built from (only meaningful while not Running).
    bool dirty() const { return editor_.revision() != builtRevision_; }
    // Rebuilds the driver from the editor's scene if dirty (see above). Fails while Running, or when the scene cannot be
    // resolved (the old driver stays).
    EditResult applyEdits();

    // The events a rebuild after an edit cut off: produced by the physics of the old simulation but not yet reached by the
    // display (see the top comment). Oldest first; handing them over empties the list. Cleared by start (from Stopped), stop and
    // loadScene.
    std::vector<Event> takeFlushedEvents() { return std::move(flushedEvents_); }

    // ---- Frames -----------------------------------------------------------------------------------------------
    // One real frame: advances the driver while Running (and pauses with lastError() set if the integrator fails); otherwise
    // applies pending edits. Returns true when the simulation was advanced.
    bool advance(double frameDt);

    // ---- Saving -----------------------------------------------------------------------------------------------
    // "Save current as scene". Running: captures the simulation now. Paused: applies pending edits, then captures. Stopped:
    // the editor's scene as it is (orbits kept, nothing lost). name empty = "<scene name> (saved)" for a run, the scene's own
    // name when Stopped.
    bool captureCurrent(Scene& out, const std::string& name = "");
    // captureCurrent + saveSceneToPath
    bool saveCurrent(const std::string& path, std::string& error, const std::string& name = "");

    // ---- The object list --------------------------------------------------------------------------------------
    // Running: the bodies and the living shuttles of the simulation now (merges and losses included). Otherwise the editor's
    // objects.
    std::vector<ObjectRow> listObjects() const;

private:
    bool rebuild(const Scene& scene, const ResolveOptions& resolve, std::string& error);
    std::uint64_t highestId() const;

    EditorModel editor_;
    RunOptions options_;
    std::unique_ptr<SimDriver> driver_;
    std::vector<std::string> buildWarnings_;
    RunState state_ = RunState::Stopped;
    Scene snapshot_;
    bool haveSnapshot_ = false;
    std::uint64_t builtRevision_ = 0;
    std::uint64_t generation_ = 0;
    std::string lastError_;
    std::vector<Event> flushedEvents_;
};

} // namespace app
} // namespace cosmos
