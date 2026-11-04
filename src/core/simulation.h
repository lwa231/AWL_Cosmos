#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "core/bodies.h"
#include "core/controllers.h"
#include "core/diagnostics.h"
#include "core/events.h"
#include "core/ias15.h"
#include "core/merge.h"
#include "core/scene.h"
#include "core/shuttles.h"
#include "core/threadpool.h"
#include "core/vec3.h"

namespace cosmos {

// Headless simulation runner (plan sections 4.4, 4.5, 4.6, 4.7, 5.4, 6.3, 6.4 and 8 items 1-3). No OpenGL, no GLFW.
//
// What it owns: the bodies and their IAS15 integrator (gravity with the 5.3 overlap field), the shuttles with their
// controllers, the event queue, the merge ledger, the simulation clock and the permanent ID allocator that bodies and
// shuttles share (an ID is never given out twice, even after the object is gone).
//
// Time model (4.4). Physics runs ahead on its own grid: advanceTo(tTarget) takes IAS15 body steps until the latest
// body step ends at or after tTarget. A step is never clipped to tTarget, so the sequence of body steps (and with it
// every number in the simulation) is the same whatever sequence of targets the caller uses; the only exception is a
// body-body contact, which clips the step it happens in to the contact time (also independent of the targets). The
// end of the latest body step is time(); the caller draws at its own display time by interpolating the dense output
// of that step (bodyDense*) and each shuttle's last step (shuttleStep, shuttlePositionAt).
//
// Per body step (4.5):
//   1. IAS15 takes one adaptive step (single-threaded). The swept test of 5.11 runs on the step with the chord between
//      the start and end states; on a contact the step is redone clipped to the contact time.
//   2. All live shuttles advance across that step against the dense output, in fixed contiguous partitions over the
//      thread pool, each task with its own event buffer. On every accepted shuttle step the swept shuttle-vs-body test
//      (surface contact, atmosphere trigger radius) runs; a shuttle that touches a body is killed at the contact point.
//   3. The per-thread events are merged and sorted by (t, id) (EventQueue::merge) into the queue.
//   4. If the bodies touched, the pair is merged (5.4) with its events, the ledger records the loss, and IAS15 is reset
//      (lazily, at the start of the next step, so the dense output of the step that ended in the merge stays readable).
//
// Determinism (4.6): the result (all body and shuttle doubles, the event list) is bit-identical for 1..16 threads and
// for any sequence of advanceTo targets that ends on the same final target. Not thread safe: use one Simulation from
// one thread (it uses its own pool internally).
//
// Dead shuttles stay in the shuttle store (alive = 0, state at their contact point) so indices stay valid; the
// Simulation never compacts the store.

struct SimulationOptions {
    int threads = 1;                    // 1..16 (ThreadPool::kMaxThreads), clamped
    double startTime = 0.0;             // t_sim at construction (the scene epoch, if you want it)
    IAS15Params ias15;                  // body integrator settings
    Dopri5Tolerance shuttleTolerance;   // error control of the shuttle gravity step
    // Keep every accepted shuttle step of the latest body step (shuttlePath), so a renderer can interpolate at any time
    // inside the step instead of only inside the last shuttle step. Costs memory proportional to the steps per body step.
    bool recordShuttlePaths = false;
};

// A sampled shuttle state, for cubic Hermite interpolation (plan 6.6).
struct ShuttleSample {
    double t = 0.0;
    Vec3 r;
    Vec3 v;
};

// The last accepted step of one shuttle: [from.t, to.t]. For a shuttle that has not stepped yet from == to == its start.
// A shuttle destroyed in its last step ends at the contact point at the contact time.
struct ShuttleLastStep {
    ShuttleSample from;
    ShuttleSample to;
};

// Cubic Hermite position between two samples (plan 6.6), t clamped to [a.t, b.t]. a.t == b.t gives a.r.
Vec3 hermitePosition(const ShuttleSample& a, const ShuttleSample& b, double t);
// Its derivative, the velocity of the interpolant.
Vec3 hermiteVelocity(const ShuttleSample& a, const ShuttleSample& b, double t);

struct SimulationStats {
    long bodySteps = 0;          // accepted IAS15 steps (including clipped ones)
    long bodyStepRedos = 0;      // steps redone clipped to a contact time
    long merges = 0;
    long shuttleSteps = 0;       // accepted shuttle steps
    long shuttleRejects = 0;
    long controllerTicks = 0;
    long shuttleForceCalls = 0;
    long shuttlesDestroyed = 0;  // ShuttleDestroyed events (surface contact or non-finite state)
};

class Simulation {
public:
    // Bodies, shuttles (with their controllers) and settings of a resolved scene. Shuttles the store rejects (invalid
    // parameters, unknown controller name) are skipped with a warning; a shuttle whose scene ID collides with a body ID
    // of the store (possible when the scene IDs were not preserved) gets a fresh ID, also with a warning.
    Simulation(const ResolvedScene& scene, const SceneSettings& settings, const SimulationOptions& options = {});
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // ---- Running ----------------------------------------------------------------------------------------------
    // Take body steps until time() >= tTarget (nothing if it already is). Returns false if the integrator failed (a
    // non-finite step it could not cure); the simulation then stays where it is and failed() is true.
    bool advanceTo(double tTarget);
    // Last target passed to advanceTo (the caller's display time; time() is at or past it).
    double targetTime() const { return target_; }
    // The end of the latest body step: the time every body, shuttle and ledger value refers to (t_sim, plan 4.4). It is
    // advanced inside IAS15 with Kahan compensated summation, so it does not drift over millions of steps.
    double time() const { return t_; }
    bool failed() const { return failed_; }

    // ---- State ------------------------------------------------------------------------------------------------
    // Bodies at time(). The store changes (an index may shift) when bodies merge; go by ID.
    const Bodies& bodies() const { return bodies_; }
    // Shuttles at time(), each at the end of its last accepted step. Indices never change.
    const Shuttles& shuttles() const { return shuttles_; }
    const std::vector<std::string>& warnings() const { return warnings_; }

    // ---- Rendering interpolation (plan 4.4, 6.6) ----------------------------------------------------------------
    // Latest body step [t0, t1] (t1 == time()). hasBodyStep() is false before the first step.
    bool hasBodyStep() const { return haveStep_; }
    double bodyStepStart() const { return stepT0_; }
    double bodyStepEnd() const { return stepT1_; }
    // The IAS15 dense output of that step: positionOf/velocityOf(i, ias.tauAt(t)) for t in [t0, t1]. Index i is the
    // body's index in bodyDenseIds() (the store order during the step; a merge at the end of the step shifts the live
    // store but not this list). Valid until the next advanceTo that takes a step.
    const IAS15& bodyDense() const { return ias_; }
    const std::vector<BodyId>& bodyDenseIds() const { return stepIds_; }
    // Convenience: position / velocity of a body of the latest step at t in [t0, t1]. false for an unknown ID or no step.
    bool bodyPositionAt(BodyId id, double t, Vec3& out) const;
    bool bodyVelocityAt(BodyId id, double t, Vec3& out) const;

    // Last accepted step of shuttle i (index into shuttles()), for cubic Hermite interpolation of the drawn position.
    const ShuttleLastStep& shuttleStep(std::size_t i) const { return lastStep_[i]; }
    // With options.recordShuttlePaths: every accepted step of the latest body step, as samples from the body step start
    // (t0) to the end (empty if the shuttle was already dead). Without it always empty.
    const std::vector<ShuttleSample>& shuttlePath(std::size_t i) const { return paths_[i]; }
    // Position (and velocity) of shuttle i at t: Hermite interpolation inside the recorded path if there is one,
    // otherwise inside the last step; t outside the available span is clamped to its ends.
    Vec3 shuttlePositionAt(std::size_t i, double t) const;
    Vec3 shuttleVelocityAt(std::size_t i, double t) const;

    // ---- Events, diagnostics -----------------------------------------------------------------------------------
    // Hand over all events since the last call, sorted by (t, id) (plan 4.5 step 3).
    std::vector<Event> drainEvents();
    const EventQueue& pendingEvents() const { return events_; }

    // Conserved quantities of the bodies now, and with the merge losses added back (plan 5.4/5.12: integrator error only).
    Conserved conserved() const { return cosmos::conserved(bodies_); }
    Conserved conservedWithMerges() const { return withMergesAddedBack(cosmos::conserved(bodies_), ledger_); }
    const MergeLedger& ledger() const { return ledger_; }
    const std::vector<MergeResult>& mergeLog() const { return mergeLog_; }
    const SimulationStats& stats() const { return stats_; }

    // ---- Settings ---------------------------------------------------------------------------------------------
    // Worker threads for the shuttle pass, clamped to 1..16. Never changes a result.
    void setThreadCount(int threads);
    int threadCount() const { return pool_.threads(); }

    // Choose the autopilot of a shuttle. Takes effect at time() (the controller is evaluated at its first step from
    // there). false for an unknown or dead shuttle.
    bool setController(ShuttleId id, ControllerKind kind);
    // The four built-in autopilots; set() replaces one (a test double, an ML link). Do not change them during advanceTo.
    ControllerSet& controllers() { return controllers_; }
    // Global infinite-fuel switch (plan 5.7).
    void setInfiniteFuel(bool on) { shuttles_.infiniteFuel = on; }
    bool infiniteFuel() const { return shuttles_.infiniteFuel; }
    // Controller tick period, s (plan 4.6); only change it while stopped, it moves the tick grid.
    void setControllerTick(double period) { controllerTick_ = period; envDirty_ = true; }
    double controllerTick() const { return controllerTick_; }

    // Add a shuttle at time() (id 0 = allocate a fresh one). Returns its ID, or 0 if the store rejected it.
    ShuttleId addShuttle(ShuttleDesc desc);
    // A fresh permanent ID, never used by any body or shuttle of this simulation.
    std::uint64_t allocateId() { return nextId_++; }

private:
    struct Impl;

    bool stepBodies();
    bool mergePair(BodyId a, BodyId b, double t);
    void advanceShuttles(double t0, double t1);
    void prepareEnvironment();
    void resizeShuttleArrays();

    Bodies bodies_;
    Shuttles shuttles_;
    ControllerSet controllers_;
    IAS15 ias_;
    IAS15 backup_;                    // copy of ias_ before a step, to redo it clipped
    ForceFn force_;
    BodyState work_;                  // end state of the step being taken

    double t_ = 0.0;
    double target_ = 0.0;
    double controllerTick_ = 0.1;
    bool failed_ = false;
    bool needReset_ = false;          // a merge happened; reset ias_ before its next step

    bool haveStep_ = false;
    double stepT0_ = 0.0, stepT1_ = 0.0;
    std::vector<BodyId> stepIds_;

    std::uint64_t nextId_ = 1;
    bool recordPaths_ = false;
    Dopri5Tolerance tolerance_;

    EventQueue events_;
    MergeLedger ledger_;
    std::vector<MergeResult> mergeLog_;
    SimulationStats stats_;
    std::vector<std::string> warnings_;

    std::vector<ShuttleLastStep> lastStep_;
    std::vector<std::vector<ShuttleSample>> paths_;

    ThreadPool pool_;
    bool envDirty_ = true;
    std::unique_ptr<Impl> impl_;      // shuttle environment, hierarchy parents, per-task scratch
};

} // namespace cosmos
