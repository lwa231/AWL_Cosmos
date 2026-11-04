#include "app/sim_driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace cosmos {
namespace app {

namespace {

SimulationOptions withShuttlePaths(SimulationOptions o)
{
    o.recordShuttlePaths = true;   // plan 6.6: Hermite between the samples of the latest body step
    return o;
}

DisplayBody propertiesOf(const Bodies& b, std::size_t i)
{
    DisplayBody d;
    d.id = b.ids[i];
    d.name = b.name[i];
    d.kind = b.kind[i];
    d.mu = b.mu[i];
    d.radius = b.radius[i];
    d.spinRate = b.spinRate[i];
    d.atmosphere = b.atmosphere[i];
    d.colour = b.colour[i];
    d.r = {b.state.x[i], b.state.y[i], b.state.z[i]};
    d.v = {b.state.vx[i], b.state.vy[i], b.state.vz[i]};
    return d;
}

const DisplayBody* findBody(const std::vector<DisplayBody>& v, BodyId id)
{
    for (const DisplayBody& d : v)
        if (d.id == id) return &d;
    return nullptr;
}

// Index of `id` in the dense-output body list, or npos. hint is tried first (the store order usually equals the list).
std::size_t denseIndex(const std::vector<BodyId>& ids, BodyId id, std::size_t hint)
{
    if (hint < ids.size() && ids[hint] == id) return hint;
    for (std::size_t k = 0; k < ids.size(); ++k)
        if (ids[k] == id) return k;
    return static_cast<std::size_t>(-1);
}

constexpr std::size_t kNoIndex = static_cast<std::size_t>(-1);

} // namespace

SimDriver::SimDriver(const ResolvedScene& scene, const SceneSettings& settings, SimulationOptions simOptions,
                     DriverOptions options)
    : sim_(scene, settings, withShuttlePaths(simOptions)),
      clock_(simOptions.startTime, options.maxFrameDt),
      options_(options)
{
    info_.target = clock_.target();
    info_.simTime = sim_.time();
    buildState(clock_.target(), state_);
}

const FrameInfo& SimDriver::advance(double frameDt)
{
    const double previous = clock_.target();
    const double warp = warp_.value();   // a change made after this call takes effect from the next frame
    clock_.advance(frameDt, warp);
    info_ = FrameInfo{};
    info_.frameDt = clock_.lastFrameDt();
    info_.warp = warp;
    return runFrame(clock_.target(), previous);
}

const FrameInfo& SimDriver::advanceTo(double t)
{
    const double previous = clock_.target();
    if (t > previous) clock_.reset(t);
    info_ = FrameInfo{};
    info_.warp = warp_.value();
    return runFrame(clock_.target(), previous);
}

const FrameInfo& SimDriver::runFrame(double wantedTarget, double previousTarget)
{
    const SimulationStats before = sim_.stats();
    runPhysics(wantedTarget);
    info_.bodySteps = sim_.stats().bodySteps - before.bodySteps;
    info_.shuttleSteps = sim_.stats().shuttleSteps - before.shuttleSteps;
    info_.bodiesSeconds = sim_.stats().bodySeconds - before.bodySeconds;
    info_.shuttleSeconds = (sim_.stats().shuttleSetupSeconds - before.shuttleSetupSeconds) + (sim_.stats().shuttleRunSeconds - before.shuttleRunSeconds);

    // The physics fell short (budget or failure): the display time waits at the physics time instead of running away
    if (sim_.time() < wantedTarget) {
        info_.lagSimSeconds = wantedTarget - sim_.time();
        clock_.holdAt(sim_.time());
    }
    info_.target = clock_.target();
    info_.simTime = sim_.time();
    info_.simDelta = info_.target - previousTarget;

    const auto stateStart = std::chrono::steady_clock::now();
    collectEvents();
    buildState(info_.target, state_);
    info_.stateSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - stateStart).count();

    // Events the display has reached. pending_ is sorted by (t, id), so they are a prefix.
    due_.clear();
    std::size_t k = 0;
    while (k < pending_.size() && pending_[k].t <= info_.target) ++k;
    due_.assign(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(k));
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(k));
    return info_;
}

void SimDriver::runPhysics(double target)
{
    if (sim_.failed()) {
        info_.failed = true;
        return;
    }
    if (sim_.time() >= target) return;   // already ahead: nothing to integrate, only interpolation

    snapshotBodies(before_);   // properties before the steps, for the merges they may contain
    using Clock = std::chrono::steady_clock;
    const Clock::time_point start = Clock::now();
    bool ok = true;
    if (options_.physicsBudget_s <= 0.0 || sim_.bodies().size() == 0) {
        ok = sim_.advanceTo(target);
    } else {
        // One body step per call (the next representable time is always crossed by exactly one step), so the budget is
        // checked between steps. The step sequence is the same as for a single advanceTo(target) (simulation.h).
        constexpr double kInf = std::numeric_limits<double>::infinity();
        while (sim_.time() < target) {
            ok = sim_.advanceTo(std::nextafter(sim_.time(), kInf));
            if (!ok || sim_.time() >= target) break;
            const std::chrono::duration<double> spent = Clock::now() - start;
            if (spent.count() >= options_.physicsBudget_s) {
                info_.physicsBehind = true;
                break;
            }
        }
    }
    info_.physicsSeconds = std::chrono::duration<double>(Clock::now() - start).count();
    if (!ok) info_.failed = true;
}

void SimDriver::snapshotBodies(std::vector<DisplayBody>& out) const
{
    const Bodies& b = sim_.bodies();
    out.resize(b.size());
    for (std::size_t i = 0; i < b.size(); ++i) out[i] = propertiesOf(b, i);
}

void SimDriver::collectEvents()
{
    drained_ = sim_.drainEvents();
    if (!drained_.empty()) {
        for (const Event& e : drained_) {
            if (e.type != EventType::Merge) continue;
            const DisplayBody* absorbed = findBody(before_, e.other);
            if (!absorbed) continue;   // cannot happen: the body was in the store before the steps
            PendingMerge pm;
            pm.survivor = e.id;
            pm.absorbed = e.other;
            pm.t = e.t;
            pm.absorbedBefore = *absorbed;
            if (const DisplayBody* s = findBody(before_, e.id)) {
                pm.survivorBefore = *s;
                pm.haveSurvivorBefore = true;
            }
            merges_.push_back(std::move(pm));
        }
        const std::size_t n = pending_.size();
        pending_.insert(pending_.end(), drained_.begin(), drained_.end());
        std::inplace_merge(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n), pending_.end(), eventBefore);
    }
    // A merge only matters while its absorbed body is still in the latest body step (merge at the end of that step)
    const std::vector<BodyId>& ids = sim_.bodyDenseIds();
    merges_.erase(std::remove_if(merges_.begin(), merges_.end(),
                                 [&](const PendingMerge& m) {
                                     return !sim_.hasBodyStep() || std::find(ids.begin(), ids.end(), m.absorbed) == ids.end();
                                 }),
                  merges_.end());
}

const SimDriver::PendingMerge* SimDriver::mergeOf(BodyId survivor) const
{
    for (const PendingMerge& m : merges_)
        if (m.survivor == survivor) return &m;
    return nullptr;
}

DisplayState SimDriver::stateAt(double t) const
{
    DisplayState s;
    buildState(t, s);
    return s;
}

void SimDriver::buildState(double t, DisplayState& out) const
{
    const Simulation& s = sim_;
    const Bodies& B = s.bodies();
    out.t = t;
    out.bodies.clear();
    out.shuttles.clear();

    // ---- Bodies: dense output of the latest body step (plan 4.4, 6.6) ----------------------------------------------
    const bool haveStep = s.hasBodyStep();
    const IAS15& dense = s.bodyDense();
    const std::vector<BodyId>& ids = s.bodyDenseIds();
    const double tau = haveStep ? dense.tauAt(std::clamp(t, s.bodyStepStart(), s.bodyStepEnd())) : 0.0;

    for (std::size_t i = 0; i < B.size(); ++i) {
        DisplayBody d = propertiesOf(B, i);   // store state at sim time; replaced below where the dense output applies
        const PendingMerge* pm = mergeOf(d.id);
        const bool beforeMerge = pm && t < pm->t;
        if (beforeMerge && pm->haveSurvivorBefore) {
            // Properties before the merge (radius, mass, colour...) so the survivor does not change before the contact
            const Vec3 r = d.r, v = d.v;
            d = pm->survivorBefore;
            d.r = r;
            d.v = v;
        }
        // After the merge time the store holds the merged body (centre of mass); the dense output still describes the pair
        const bool useDense = haveStep && !(pm && !beforeMerge);
        if (useDense) {
            const std::size_t k = denseIndex(ids, d.id, i);
            if (k != kNoIndex) {
                d.r = dense.positionOf(k, tau);
                d.v = dense.velocityOf(k, tau);
                d.interpolated = true;
            }
        }
        out.bodies.push_back(std::move(d));
    }
    // Bodies absorbed by a merge that the display has not reached: still drawn, from the dense output
    if (haveStep) {
        for (const PendingMerge& m : merges_) {
            if (!(t < m.t)) continue;
            const std::size_t k = denseIndex(ids, m.absorbed, kNoIndex);
            if (k == kNoIndex) continue;
            DisplayBody d = m.absorbedBefore;
            d.r = dense.positionOf(k, tau);
            d.v = dense.velocityOf(k, tau);
            d.interpolated = true;
            d.absorbed = true;
            out.bodies.push_back(std::move(d));
        }
    }

    // ---- Shuttles: cubic Hermite from the samples of the latest body step ------------------------------------------
    const Shuttles& S = s.shuttles();
    for (std::size_t i = 0; i < S.size(); ++i) {
        const ShuttleLastStep& ls = s.shuttleStep(i);
        bool dying = false;
        if (!S.alive[i]) {
            // Destroyed in the latest step: visible until the contact time. Destroyed earlier: its contact time is
            // before this step, so the test fails and it is not drawn.
            if (!(t < ls.to.t)) continue;
            dying = true;
        }
        DisplayShuttle d;
        d.id = S.ids[i];
        d.index = i;
        d.dying = dying;
        d.r = s.shuttlePositionAt(i, t);
        d.v = s.shuttleVelocityAt(i, t);
        d.interpolated = s.shuttlePath(i).size() >= 2 || ls.to.t > ls.from.t;
        d.throttle = S.throttle[i];
        // Heading and spin: Hermite of the recorded samples, like the position (the torque is constant between samples,
        // so this is exact there and continuous across steps). Purely visual.
        d.heading = s.shuttleHeadingAt(i, t);
        d.spin = s.shuttleSpinAt(i, t);
        out.shuttles.push_back(d);
    }
}

} // namespace app
} // namespace cosmos
