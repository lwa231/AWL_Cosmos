#include "core/simulation.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include "core/collisions.h"
#include "core/hierarchy.h"
#include "core/impact.h"

namespace cosmos {

// ---------------------------------------------------------------------------------------------------------------
// Cubic Hermite interpolation (plan 6.6)

Vec3 hermitePosition(const ShuttleSample& a, const ShuttleSample& b, double t)
{
    const double h = b.t - a.t;
    if (!(h > 0.0)) return a.r;
    const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    return a.r * h00 + a.v * (h10 * h) + b.r * h01 + b.v * (h11 * h);
}

Vec3 hermiteVelocity(const ShuttleSample& a, const ShuttleSample& b, double t)
{
    const double h = b.t - a.t;
    if (!(h > 0.0)) return a.v;
    const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
    const double s2 = s * s;
    const double d00 = (6.0 * s2 - 6.0 * s) / h;
    const double d10 = 3.0 * s2 - 4.0 * s + 1.0;
    const double d01 = (-6.0 * s2 + 6.0 * s) / h;
    const double d11 = 3.0 * s2 - 2.0 * s;
    return a.r * d00 + a.v * d10 + b.r * d01 + b.v * d11;
}

namespace {

constexpr std::size_t kNpos = static_cast<std::size_t>(-1);
// Shuttle work is cut into this many contiguous index ranges whatever the thread count (plan 4.5/4.6): the cut points
// depend only on the number of shuttles, never on how many threads run the tasks.
constexpr std::size_t kShuttleTasks = 64;
// A body step is redone clipped to a contact at most this many times (each redo can reveal an earlier contact).
constexpr int kMaxBodyRedos = 8;
// A contact this close to the end of the step (relative to its length) counts as at the end: no redo.
constexpr double kEndContactFraction = 1e-9;

// ---------------------------------------------------------------------------------------------------------------
// Parent node of a shuttle (plan 5.9, 15 item 2c): the hierarchy of hierarchy.h, built once per body step from the
// state at the start of the step, queried at any time inside the step.
//
// The nodes move while the step runs, so the Hill-sphere test of shuttleParent (same rule: deepest node whose finite
// Hill sphere contains the shuttle, ties by distance, else the root) is evaluated against the nodes' positions at the
// query time (start position + velocity * elapsed). The Hill radii and depths are computed once per build. The state
// of the chosen node is exact: a body from the dense output, a barycentre from its children's.
class HierarchyParents final : public IParentProvider {
public:
    void build(const Bodies& b, const IBodyEphemeris& eph, double t0)
    {
        eph_ = &eph;
        t0_ = t0;
        ids_ = b.ids;
        r_.resize(b.size());
        v_.resize(b.size());
        for (std::size_t i = 0; i < b.size(); ++i) {
            r_[i] = {b.state.x[i], b.state.y[i], b.state.z[i]};
            v_[i] = {b.state.vx[i], b.state.vy[i], b.state.vz[i]};
        }
        h_ = buildHierarchy(r_, v_, b.mu);
        const std::size_t m = h_.nodes.size();
        hill_.assign(m, 0.0);
        depth_.assign(m, 0);
        for (std::size_t i = 0; i < m; ++i) {
            hill_[i] = hillRadius(h_, static_cast<int>(i));
            int d = 0;
            for (int g = static_cast<int>(i); h_.nodes[static_cast<std::size_t>(g)].parent != -1;
                 g = h_.nodes[static_cast<std::size_t>(g)].parent)
                ++d;
            depth_[i] = d;
        }
    }

    ParentRef parentOf(double t, const Vec3& position) const override
    {
        ParentRef out;
        if (h_.root < 0) return out;
        const double dt = t - t0_;
        int best = h_.root;
        int bestDepth = 0;
        double bestDist = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < h_.nodes.size(); ++i) {
            if (static_cast<int>(i) == h_.root) continue;
            const double rh = hill_[i];
            if (!std::isfinite(rh)) continue;
            const HierarchyNode& n = h_.nodes[i];
            const double d = norm(position - (n.r + n.v * dt));
            if (d >= rh) continue;
            if (depth_[i] > bestDepth || (depth_[i] == bestDepth && d < bestDist)) {
                best = static_cast<int>(i);
                bestDepth = depth_[i];
                bestDist = d;
            }
        }
        const HierarchyNode& n = h_.nodes[static_cast<std::size_t>(best)];
        out.mu = n.mu;
        out.id = static_cast<std::size_t>(best) < ids_.size() ? ids_[static_cast<std::size_t>(best)] : 0;
        nodeState(best, t, out.r, out.v);
        return out;
    }

private:
    // Barycentre state of a node at time t from the bodies' ephemeris (a leaf is the body itself).
    void nodeState(int i, double t, Vec3& r, Vec3& v) const
    {
        const HierarchyNode& n = h_.nodes[static_cast<std::size_t>(i)];
        if (n.childA < 0) {
            r = eph_->position(static_cast<std::size_t>(i), t);
            v = eph_->velocity(static_cast<std::size_t>(i), t);
            return;
        }
        Vec3 ra, va, rb, vb;
        nodeState(n.childA, t, ra, va);
        nodeState(n.childB, t, rb, vb);
        const double muA = h_.nodes[static_cast<std::size_t>(n.childA)].mu;
        const double muB = h_.nodes[static_cast<std::size_t>(n.childB)].mu;
        if (n.mu > 0.0) {
            r = (ra * muA + rb * muB) / n.mu;
            v = (va * muA + vb * muB) / n.mu;
        } else {
            r = (ra + rb) * 0.5;
            v = (va + vb) * 0.5;
        }
    }

    const IBodyEphemeris* eph_ = nullptr;
    double t0_ = 0.0;
    std::vector<BodyId> ids_;
    std::vector<Vec3> r_, v_;
    Hierarchy h_;
    std::vector<double> hill_;
    std::vector<int> depth_;
};

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// Private implementation

struct Simulation::Impl {
    explicit Impl(const IAS15& ias) : ephemeris(ias) {}

    Ias15Ephemeris ephemeris;
    std::optional<ShuttleEnvironment> env;
    HierarchyParents parents;

    // Per task (index = task number, fixed by the shuttle count): event buffer and scratch of the swept hook.
    struct TaskScratch {
        std::vector<ShuttleBodyHit> hits;
        std::vector<Vec3> endPos, prevEndPos;   // body positions at the end of the shuttle's previous / this step
    };
    std::vector<std::vector<Event>> taskEvents;
    std::vector<ShuttleStats> taskStats;
    std::vector<TaskScratch> scratch;
};

namespace {

// The swept shuttle-vs-body test run after every accepted shuttle step (plan 5.11, 8 items 1-3; 4.7). Called on the
// worker thread that runs the shuttle; everything it writes belongs to that shuttle (its entries of lastStep/paths) or
// to the task (events, scratch).
class SweptHook final : public IShuttleStepHook {
public:
    SweptHook(const ShuttleEnvironment& env, const Shuttles& shuttles, std::vector<ShuttleLastStep>& lastStep,
              std::vector<std::vector<ShuttleSample>>& paths, bool recordPaths, std::vector<ShuttleBodyHit>& hits,
              std::vector<Vec3>& endPos, std::vector<Vec3>& prevEndPos)
        : env_(env), shuttles_(shuttles), lastStep_(lastStep), paths_(paths), record_(recordPaths), hits_(hits),
          endPos_(endPos), prevEnd_(prevEndPos)
    {
    }

    // Set before each shuttle: no body positions cached, no kill pending.
    void beginShuttle()
    {
        killed = false;
        cachedShuttle_ = kNpos;
    }

    bool killed = false;       // the shuttle touched a body in the step just reported
    double killT = 0.0;        // contact time (sim s)
    Vec3 killPoint, killVelocity;

    bool onAcceptedStep(const ShuttleStepInfo& s, std::vector<Event>& events) override
    {
        const Bodies& B = env_.bodies;
        const IBodyEphemeris& eph = env_.ephemeris;
        const std::size_t n = B.size();
        const double dt = s.t1 - s.t0;
        const double rs = shuttles_.radius[s.index];

        auto& hits = hits_;
        auto& endPos = endPos_;
        auto& prevEnd = prevEnd_;
        endPos.resize(n);
        // The positions at the end of the previous step of this shuttle are this step's start positions: reuse them
        // (the same function of the same time, so the numbers are identical to a fresh evaluation).
        const bool reuse = cachedShuttle_ == s.index && cachedT_ == s.t0 && prevEnd.size() == n;

        hits.clear();
        for (std::size_t j = 0; j < n; ++j) {
            const Vec3 b0 = reuse ? prevEnd[j] : eph.position(j, s.t0);
            const Vec3 b1 = eph.position(j, s.t1);
            endPos[j] = b1;
            const SweptShuttleBody c = sweptShuttleVsBody(s.r0, s.r1, b0, b1, dt, B.radius[j], rs, env_.atmosphereTop[j]);
            if (c.surface || c.atmosphere) hits.push_back({j, B.ids[j], c});
        }
        std::swap(endPos, prevEnd);
        cachedShuttle_ = s.index;
        cachedT_ = s.t1;

        // The body that destroys the shuttle: earliest surface contact, ties by smaller body id (as in
        // appendShuttleStepEvents, which also drops the atmosphere entries after it).
        const ShuttleBodyHit* killer = nullptr;
        for (const ShuttleBodyHit& h : hits) {
            if (!h.contact.surface) continue;
            if (!killer || *h.contact.surface < *killer->contact.surface ||
                (*h.contact.surface == *killer->contact.surface && h.body < killer->body))
                killer = &h;
        }
        if (!hits.empty()) appendShuttleStepEvents(events, s.t0, s.id, hits);

        ShuttleSample from{s.t0, s.r0, s.v0};
        ShuttleSample to{s.t1, s.r1, s.v1};
        if (killer) {
            // Destroyed at the contact point: the chord position and the velocity interpolated the same way.
            const double tc = *killer->contact.surface;
            const double f = dt > 0.0 ? std::min(tc / dt, 1.0) : 0.0;
            killed = true;
            killT = s.t0 + tc;
            killPoint = killer->contact.surfacePoint;
            killVelocity = s.v0 + (s.v1 - s.v0) * f;
            to = {killT, killPoint, killVelocity};
        }
        lastStep_[s.index] = {from, to};
        if (record_) paths_[s.index].push_back(to);
        return killer == nullptr;
    }

private:
    const ShuttleEnvironment& env_;
    const Shuttles& shuttles_;
    std::vector<ShuttleLastStep>& lastStep_;
    std::vector<std::vector<ShuttleSample>>& paths_;
    bool record_;
    std::vector<ShuttleBodyHit>& hits_;
    std::vector<Vec3>& endPos_;
    std::vector<Vec3>& prevEnd_;
    std::size_t cachedShuttle_ = kNpos;
    double cachedT_ = 0.0;
};

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// Construction

Simulation::Simulation(const ResolvedScene& scene, const SceneSettings& settings, const SimulationOptions& options)
    : bodies_(scene.bodies),
      ias_(options.ias15),
      backup_(options.ias15),
      t_(options.startTime),
      target_(options.startTime),
      controllerTick_(settings.controllerTick_s),
      recordPaths_(options.recordShuttlePaths),
      tolerance_(options.shuttleTolerance),
      pool_(options.threads),
      impl_(new Impl(ias_))
{
    force_ = [this](const BodyState& s, Accel& a) { computeAccelerations(s, bodies_.mu, bodies_.radius, a); };
    shuttles_.infiniteFuel = settings.infiniteFuel;
    if (!(controllerTick_ > 0.0) || !std::isfinite(controllerTick_)) {
        warnings_.push_back("controller tick period must be positive; using 0.1 s");
        controllerTick_ = 0.1;
    }

    // IDs are permanent: the allocator starts above every ID in the scene (bodies and shuttles share one number line).
    std::uint64_t top = 0;
    for (BodyId id : bodies_.ids) top = std::max<std::uint64_t>(top, id);
    for (const ResolvedShuttle& r : scene.shuttles) top = std::max<std::uint64_t>(top, r.desc.id);
    nextId_ = top + 1;

    for (const ResolvedShuttle& r : scene.shuttles) {
        ShuttleDesc d;
        if (!shuttleDescFromResolved(r, d)) {
            warnings_.push_back("shuttle '" + r.desc.name + "' skipped: unknown controller '" + r.desc.controller + "'");
            continue;
        }
        if (d.id == 0 || bodies_.find(d.id) || shuttles_.find(d.id)) {
            const std::uint64_t fresh = allocateId();
            warnings_.push_back("shuttle '" + r.desc.name + "': ID " + std::to_string(d.id) + " is not free, using " +
                                std::to_string(fresh));
            d.id = fresh;
        }
        if (!shuttles_.add(d)) warnings_.push_back("shuttle '" + r.desc.name + "' skipped: invalid parameters");
    }
    resizeShuttleArrays();
}

Simulation::~Simulation() = default;

void Simulation::resizeShuttleArrays()
{
    const std::size_t old = lastStep_.size();
    lastStep_.resize(shuttles_.size());
    paths_.resize(shuttles_.size());
    for (std::size_t i = old; i < shuttles_.size(); ++i) {
        const ShuttleSample s{t_, {shuttles_.state.x[i], shuttles_.state.y[i], shuttles_.state.z[i]},
                              {shuttles_.state.vx[i], shuttles_.state.vy[i], shuttles_.state.vz[i]}};
        lastStep_[i] = {s, s};
    }
}

ShuttleId Simulation::addShuttle(ShuttleDesc desc)
{
    if (desc.id == 0) desc.id = nextId_;
    if (bodies_.find(desc.id) || shuttles_.find(desc.id)) return 0;
    if (!shuttles_.add(desc)) return 0;
    nextId_ = std::max<std::uint64_t>(nextId_, desc.id + 1);
    resizeShuttleArrays();
    return desc.id;
}

bool Simulation::setController(ShuttleId id, ControllerKind kind)
{
    const auto i = shuttles_.find(id);
    if (!i || shuttles_.alive[*i] == 0) return false;
    shuttles_.controller[*i] = kind;
    shuttles_.nextTick[*i] = kTickUnset; // evaluate the new controller at the start of the next step
    return true;
}

void Simulation::setThreadCount(int threads) { pool_.resize(threads); }

// ---------------------------------------------------------------------------------------------------------------
// Running

bool Simulation::advanceTo(double tTarget)
{
    target_ = tTarget;
    if (failed_) return false;
    if (bodies_.size() == 0) {
        // Nothing to integrate (and nothing for the shuttles to feel): the clock just runs.
        if (tTarget > t_) t_ = tTarget;
        return true;
    }
    while (t_ < tTarget) {
        if (!stepBodies()) return false;
    }
    if (!std::is_sorted(events_.events().begin(), events_.events().end(), eventBefore)) events_.sort();
    return true;
}

bool Simulation::mergePair(BodyId a, BodyId b, double t)
{
    const std::optional<MergeResult> m = mergeBodies(bodies_, a, b, t, &events_);
    if (!m) return false;
    record(ledger_, *m);
    mergeLog_.push_back(*m);
    ++stats_.merges;
    needReset_ = true;   // the IAS15 history belongs to the old set of bodies (reset lazily: see the header)
    envDirty_ = true;
    return true;
}

bool Simulation::stepBodies()
{
    if (needReset_) {
        ias_.reset();
        needReset_ = false;
    }
    const double t0 = t_;
    backup_ = ias_;
    work_ = bodies_.state;
    double t = t0;

    auto fail = [&](const char* what) {
        failed_ = true;
        warnings_.push_back(what);
        return false;
    };

    // 1. One unclipped adaptive step (plan 4.4: the step sequence never depends on the caller's targets).
    double dt = ias_.step(work_, force_, t);
    if (!(dt > 0.0)) return fail(ias_.lastFailed() ? "body integrator failed: non-finite step" : "body integrator made no progress");
    ++stats_.bodySteps;

    // 2. Swept body-body test with the chord from the start state to the end state (plan 5.11).
    std::vector<BodyContact> contacts = findBodyContacts(bodies_, work_, t - t0);
    std::optional<std::pair<BodyId, BodyId>> forced; // the pair that clipped the step: merged at its end whatever the retest says
    for (int redo = 0; redo < kMaxBodyRedos && !contacts.empty() && contacts.front().t < (t - t0) * (1.0 - kEndContactFraction);
         ++redo) {
        const BodyContact first = contacts.front();
        const double tc = t0 + first.t;
        if (!(tc > t0 + 4.0 * (std::nextafter(std::fabs(t0), INFINITY) - std::fabs(t0)))) {
            // Touching (or overlapping) at the start already: merge now without taking a step.
            ias_ = backup_;
            for (const BodyContact& c : contacts) {
                if (c.t > 0.0) break;
                mergePair(c.a, c.b, t0);
            }
            return true;
        }
        // Redo the step from the saved integrator state, clipped to the contact time.
        ias_ = backup_;
        work_ = bodies_.state;
        t = t0;
        dt = ias_.step(work_, force_, t, tc);
        if (!(dt > 0.0)) return fail("body integrator failed while clipping a step to a contact");
        ++stats_.bodyStepRedos;
        forced = std::make_pair(first.a, first.b);
        contacts = findBodyContacts(bodies_, work_, t - t0);
    }

    // 3. Shuttles across the same body step, against the dense output (plan 4.5).
    const double s0 = ias_.stepStartTime();
    const double s1 = ias_.stepEndTime();
    stepT0_ = s0;
    stepT1_ = s1;
    stepIds_ = bodies_.ids;
    haveStep_ = true;
    advanceShuttles(s0, s1);

    // 4. Commit the bodies and merge the pairs that touch at the end of the step.
    bodies_.state = work_;
    t_ = t;
    if (forced) mergePair(forced->first, forced->second, t_);
    for (const BodyContact& c : contacts) mergePair(c.a, c.b, t_); // a pair already merged above no longer resolves
    return true;
}

void Simulation::prepareEnvironment()
{
    Impl& I = *impl_;
    if (!I.env || envDirty_) {
        I.env.emplace(bodies_, I.ephemeris, controllers_);
        envDirty_ = false;
    }
    ShuttleEnvironment& env = *I.env;
    env.controllerTick = controllerTick_;
    env.tolerance = tolerance_;
    env.parents = &I.parents;
}

void Simulation::advanceShuttles(double t0, double t1)
{
    const std::size_t n = shuttles_.size();
    if (recordPaths_) {
        for (std::size_t i = 0; i < n; ++i) {
            paths_[i].clear();
            if (shuttles_.alive[i] != 0)
                paths_[i].push_back({t0, {shuttles_.state.x[i], shuttles_.state.y[i], shuttles_.state.z[i]},
                                     {shuttles_.state.vx[i], shuttles_.state.vy[i], shuttles_.state.vz[i]}});
        }
    }
    if (n == 0 || shuttles_.aliveCount() == 0) return;

    prepareEnvironment();
    Impl& I = *impl_;
    I.parents.build(bodies_, I.ephemeris, t0); // bodies_.state is still the state at the start of the step
    const ShuttleEnvironment& env = *I.env;

    const std::size_t tasks = std::min(n, kShuttleTasks);
    I.taskEvents.resize(tasks);
    I.taskStats.assign(tasks, ShuttleStats{});
    I.scratch.resize(tasks);
    for (auto& e : I.taskEvents) e.clear();

    pool_.run(tasks, [&](std::size_t k) {
        const std::size_t lo = k * n / tasks, hi = (k + 1) * n / tasks;
        Impl::TaskScratch& sc = I.scratch[k];
        SweptHook hook(env, shuttles_, lastStep_, paths_, recordPaths_, sc.hits, sc.endPos, sc.prevEndPos);
        for (std::size_t i = lo; i < hi; ++i) {
            if (shuttles_.alive[i] == 0) continue;
            hook.beginShuttle();
            propagateShuttle(shuttles_, i, t0, t1, env, I.taskEvents[k], &hook, &I.taskStats[k]);
            if (shuttles_.alive[i] == 0 && hook.killed) {
                // Destroyed on touching a body: the shuttle ends at the contact point (the hook recorded it)
                shuttles_.state.x[i] = hook.killPoint.x;
                shuttles_.state.y[i] = hook.killPoint.y;
                shuttles_.state.z[i] = hook.killPoint.z;
                shuttles_.state.vx[i] = hook.killVelocity.x;
                shuttles_.state.vy[i] = hook.killVelocity.y;
                shuttles_.state.vz[i] = hook.killVelocity.z;
            }
        }
    });

    // Per-thread buffers into one queue sorted by (t, id) (plan 4.5 step 3)
    const EventQueue merged = EventQueue::merge(I.taskEvents);
    for (const Event& e : merged.events())
        if (e.type == EventType::ShuttleDestroyed) ++stats_.shuttlesDestroyed;
    events_.append(merged.events());
    for (const ShuttleStats& s : I.taskStats) {
        stats_.shuttleSteps += s.acceptedSteps;
        stats_.shuttleRejects += s.rejectedSteps;
        stats_.controllerTicks += s.controllerTicks;
        stats_.shuttleForceCalls += s.forceCalls;
    }
}

std::vector<Event> Simulation::drainEvents()
{
    if (!std::is_sorted(events_.events().begin(), events_.events().end(), eventBefore)) events_.sort();
    return events_.drain();
}

// ---------------------------------------------------------------------------------------------------------------
// Interpolation helpers

bool Simulation::bodyPositionAt(BodyId id, double t, Vec3& out) const
{
    if (!haveStep_) return false;
    const auto it = std::find(stepIds_.begin(), stepIds_.end(), id);
    if (it == stepIds_.end()) return false;
    out = ias_.positionOf(static_cast<std::size_t>(it - stepIds_.begin()), ias_.tauAt(std::clamp(t, stepT0_, stepT1_)));
    return true;
}

bool Simulation::bodyVelocityAt(BodyId id, double t, Vec3& out) const
{
    if (!haveStep_) return false;
    const auto it = std::find(stepIds_.begin(), stepIds_.end(), id);
    if (it == stepIds_.end()) return false;
    out = ias_.velocityOf(static_cast<std::size_t>(it - stepIds_.begin()), ias_.tauAt(std::clamp(t, stepT0_, stepT1_)));
    return true;
}

namespace {
// The pair of samples around t in a recorded path (t clamped to the ends).
void bracket(const std::vector<ShuttleSample>& path, double t, const ShuttleSample*& a, const ShuttleSample*& b)
{
    if (path.size() == 1) {
        a = b = &path[0];
        return;
    }
    const auto it = std::upper_bound(path.begin(), path.end(), t, [](double v, const ShuttleSample& s) { return v < s.t; });
    if (it == path.begin()) {
        a = &path[0];
        b = &path[1];
    } else if (it == path.end()) {
        a = &path[path.size() - 2];
        b = &path[path.size() - 1];
    } else {
        a = &*(it - 1);
        b = &*it;
    }
}
} // namespace

Vec3 Simulation::shuttlePositionAt(std::size_t i, double t) const
{
    if (recordPaths_ && !paths_[i].empty()) {
        const ShuttleSample *a, *b;
        bracket(paths_[i], t, a, b);
        return hermitePosition(*a, *b, t);
    }
    return hermitePosition(lastStep_[i].from, lastStep_[i].to, t);
}

Vec3 Simulation::shuttleVelocityAt(std::size_t i, double t) const
{
    if (recordPaths_ && !paths_[i].empty()) {
        const ShuttleSample *a, *b;
        bracket(paths_[i], t, a, b);
        return hermiteVelocity(*a, *b, t);
    }
    return hermiteVelocity(lastStep_[i].from, lastStep_[i].to, t);
}

} // namespace cosmos
