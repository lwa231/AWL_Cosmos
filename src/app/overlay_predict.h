#pragma once
// Predicted path (plan 9.5): a worker thread integrates a COPY of the simulation forward for N seconds of sim time and the
// result is drawn. GL-free.
//
//  - request() takes a snapshot on the calling thread (a copy of the body store at sim.time() and, for a shuttle target,
//    that one shuttle's state, masses and autopilot) and returns at once. The live Simulation is only read during that
//    call and is never referenced by the worker: the worker builds its own Simulation from the snapshot, so it cannot
//    touch the live one (tested bit for bit).
//  - A new request supersedes the old one: the worker notices at the next sample and drops the old job, its result is
//    discarded. The frame never waits: poll() is a try-lock and a pointer swap.
//  - The copy is a fresh Simulation (same integrators, one thread) started at t0. Other shuttles are left out: they are
//    massless and cannot change the target's path. A body target is propagated with the bodies only.
//  - Samples: positions of the target and of every body at t0 + k * horizon / samples, k = 0..samples, taken from the
//    IAS15 dense output and the shuttle's recorded path, so they are exact at the sample times (the same numbers a
//    direct propagation gives). Bodies are kept so the path can be re-expressed in any reference frame at draw time.
//  - Work is bounded: a job stops after `maxBodySteps` IAS15 steps (the path is then truncated, truncated = true). It also
//    stops when the target is destroyed or absorbed (targetEnded) or the integrator fails.
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include "app/camera.h"
#include "core/simulation.h"

namespace cosmos {
namespace app {

struct PredictRequest {
    TargetKind kind = TargetKind::None;   // Body or Shuttle
    std::uint64_t id = 0;
    double horizon_s = 0.0;               // sim seconds to look ahead (> 0)
    int samples = 512;                    // intervals; the path has samples + 1 points
    std::size_t maxBodySteps = 200000;
    SimulationOptions sim;                // integrator settings of the copy; threads, startTime and recordShuttlePaths are overridden
};

struct PredictedPath {
    std::uint64_t requestId = 0;          // 0 = no result
    TargetKind kind = TargetKind::None;
    std::uint64_t id = 0;
    double t0 = 0.0;                      // sim time of the snapshot
    double dt = 0.0;                      // sample k is at t0 + k * dt
    std::size_t count = 0;                // valid samples
    bool truncated = false;               // stopped early: step budget, integrator failure or superseded mid-way
    bool targetEnded = false;             // the target was destroyed or absorbed; the last sample is where it ended
    std::vector<Vec3> target;             // count entries: inertial position of the target
    std::vector<std::uint64_t> bodyIds;   // bodies at t0 (columns of `bodies`)
    std::vector<Vec3> bodies;             // count * bodyIds.size(): row k, column j; NaN x for a body that is gone
    std::vector<Vec3> barycentre;         // count entries: mu-weighted barycentre of the bodies still present

    bool valid() const { return requestId != 0 && count >= 2; }
    double timeOf(std::size_t k) const { return t0 + dt * static_cast<double>(k); }
    // Column of a body ID in `bodies`, or -1.
    int bodyColumn(std::uint64_t bodyId) const;
};

// What the worker needs, copied from the live simulation (plain data, safe to move to another thread).
struct PredictSnapshot {
    Bodies bodies;
    bool hasShuttle = false;
    ShuttleDesc shuttle;
    SceneSettings settings;
    double t0 = 0.0;
};

// Copies the state at sim.time(). False if the target does not exist or is a dead shuttle.
bool makeSnapshot(const Simulation& sim, TargetKind kind, std::uint64_t id, PredictSnapshot& out);

// The job itself, synchronous. `abort` (optional) is polled between sample chunks; when it returns true the run stops and
// `out.truncated` is set. Always fills `out` (count may be 0).
void runPrediction(const PredictSnapshot& snap, const PredictRequest& req, PredictedPath& out,
                   bool (*abort)(const void*) = nullptr, const void* abortArg = nullptr);

// Convenience for tests and tools: snapshot + run on the calling thread. requestId is set to 1.
PredictedPath predictNow(const Simulation& sim, const PredictRequest& req);

class PathPredictor {
public:
    PathPredictor();
    ~PathPredictor();   // asks the worker to stop and joins it (at most one sample chunk of work)
    PathPredictor(const PathPredictor&) = delete;
    PathPredictor& operator=(const PathPredictor&) = delete;

    // Snapshot + queue. Returns the request number (>= 1), or 0 if the target is not available (nothing queued).
    std::uint64_t request(const Simulation& sim, const PredictRequest& req);
    // Take the newest finished result if there is one (and it belongs to the latest request). Never blocks beyond a
    // try-lock. Returns true when result() changed.
    bool poll();
    // The latest result handed over by poll(); requestId == 0 until the first one. Main-thread data: the worker never
    // touches it.
    const PredictedPath& result() const { return latest_; }
    // Number of the latest request made (0 if none).
    std::uint64_t latestRequest() const;
    // A request is queued or running.
    bool busy() const;
    // Forget the queued/running request and drop the result.
    void cancel();
    // Wait until no request is queued or running; false on timeout. For tests; the frame never calls it.
    bool waitIdle(double timeoutSeconds);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    PredictedPath latest_;
};

} // namespace app
} // namespace cosmos
