#include "core/simulation.h"
#include <algorithm>
#include <chrono>
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

double hermiteHeading(const ShuttleSample& a, const ShuttleSample& b, double t)
{
    const double h = b.t - a.t;
    if (!(h > 0.0)) return a.heading;
    const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s;
    return a.heading * (2.0 * s3 - 3.0 * s2 + 1.0) + a.spin * (h * (s3 - 2.0 * s2 + s)) +
           b.heading * (-2.0 * s3 + 3.0 * s2) + b.spin * (h * (s3 - s2));
}

double hermiteSpin(const ShuttleSample& a, const ShuttleSample& b, double t)
{
    const double h = b.t - a.t;
    if (!(h > 0.0)) return a.spin;
    const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
    const double s2 = s * s;
    return a.heading * ((6.0 * s2 - 6.0 * s) / h) + a.spin * (3.0 * s2 - 4.0 * s + 1.0) +
           b.heading * ((-6.0 * s2 + 6.0 * s) / h) + b.spin * (3.0 * s2 - 2.0 * s);
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
// Consecutive body steps that neither advance time nor merge before advanceTo gives up.
constexpr int kMaxStalledSteps = 8;
// Two bodies whose centres are within (R_a + R_b)(1 + this) count as touching.
constexpr double kTouchTolerance = 1e-9;

Vec3 bodyPos(const Bodies& b, std::size_t i) { return {b.state.x[i], b.state.y[i], b.state.z[i]}; }

// ---------------------------------------------------------------------------------------------------------------
// Parent node of a shuttle (plan 5.9, 15 item 2c): the hierarchy of hierarchy.h, built once per body step from the
// state at the start of the step, queried at any time inside the step.
//
// The nodes move while the step runs, so the Hill-sphere test of shuttleParent (same rule: smallest finite Hill sphere
// that contains the shuttle, ties by betterParent, else the root) is evaluated against the nodes' positions at the
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
        ParentCandidate bestC;
        bool have = false;
        for (std::size_t i = 0; i < h_.nodes.size(); ++i) {
            if (static_cast<int>(i) == h_.root) continue;
            const double rh = hill_[i];
            if (!std::isfinite(rh)) continue;
            const HierarchyNode& n = h_.nodes[i];
            const double d = norm(position - (n.r + n.v * dt));
            if (d >= rh) continue;
            const ParentCandidate c{rh, n.childA < 0, depth_[i], d};
            if (!have || betterParent(c, bestC)) {
                best = static_cast<int>(i);
                bestC = c;
                have = true;
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
    // The dense output of the latest body step, copied out of ias_ for the shuttle pass (DenseEphemeris::build per body step).
    DenseEphemeris ephemeris;
    std::optional<ShuttleEnvironment> env;
    HierarchyParents parents;

    // Per task (index = task number, fixed by the shuttle count): event buffer and scratch of the swept hook.
    struct TaskScratch {
        std::vector<ShuttleBodyHit> hits;
        std::vector<Vec3> endPos, prevEndPos;   // body positions at the end of the shuttle's previous / this step
        ShuttleScratch work;                    // propagateShuttle's scratch: body positions at the last time asked for, ...
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
              std::vector<Vec3>& endPos, std::vector<Vec3>& prevEndPos, BodyPositionCache& cache)
        : env_(env), shuttles_(shuttles), lastStep_(lastStep), paths_(paths), record_(recordPaths), hits_(hits),
          endPos_(endPos), prevEnd_(prevEndPos), cache_(cache)
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
        const Vec3* end = cache_.at(eph, s.t1);   // the propagator evaluated these for its last stage: no new evaluation
        for (std::size_t j = 0; j < n; ++j) {
            const Vec3 b0 = reuse ? prevEnd[j] : eph.position(j, s.t0);
            const Vec3 b1 = end[j];
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
        // A FuelEmpty that the propagator pushed for this step just before calling the hook, later than the destruction,
        // is dropped: the shuttle is gone by then.
        if (killer && !events.empty()) {
            const Event& last = events.back();
            if (last.type == EventType::FuelEmpty && last.id == s.id && last.t > s.t0 + *killer->contact.surface)
                events.pop_back();
        }
        if (!hits.empty()) appendShuttleStepEvents(events, s.t0, s.id, hits);

        ShuttleSample from{s.t0, s.r0, s.v0, s.heading0, s.spin0};
        ShuttleSample to{s.t1, s.r1, s.v1, s.heading1, s.spin1};
        if (killer) {
            // Destroyed at the contact point: the chord position and the velocity interpolated the same way.
            const double tc = *killer->contact.surface;
            const double f = dt > 0.0 ? std::min(tc / dt, 1.0) : 0.0;
            killed = true;
            killT = s.t0 + tc;
            killPoint = killer->contact.surfacePoint;
            killVelocity = s.v0 + (s.v1 - s.v0) * f;
            to = {killT, killPoint, killVelocity, s.heading0 + (s.heading1 - s.heading0) * f,
                  s.spin0 + (s.spin1 - s.spin0) * f};
        }
        lastStep_[s.index] = {from, to};
        if (record_) {
            // The step's own headings are relative to its start (wrapped there); chain them onto the previous sample so
            // the whole path is continuous.
            std::vector<ShuttleSample>& path = paths_[s.index];
            if (s.rotationCount > 0 && !killer) {
                // Controller ticks inside the step (a shuttle that cannot thrust): one sample per tick with the exact heading
                // and spin; the position and velocity there come from the cubic Hermite of the step, which is what the
                // display would use without these samples (a Hermite through samples of a cubic is that cubic).
                double prevHeading = from.heading;
                for (std::size_t m = 0; m < s.rotationCount; ++m) {
                    const ShuttleRotationSample& q = s.rotations[m];
                    ShuttleSample p;
                    p.t = q.t;
                    p.r = hermitePosition(from, to, q.t);
                    p.v = hermiteVelocity(from, to, q.t);
                    p.heading = (path.empty() ? from.heading : path.back().heading) + (q.heading - prevHeading);
                    p.spin = q.spin;
                    prevHeading = q.heading;
                    path.push_back(p);
                }
                ShuttleSample p = to;
                p.heading = (path.empty() ? from.heading : path.back().heading) + (to.heading - prevHeading);
                path.push_back(p);
            } else {
                ShuttleSample p = to;
                if (!path.empty()) p.heading = path.back().heading + (to.heading - from.heading);
                path.push_back(p);
            }
        }
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
    BodyPositionCache& cache_;
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
      impl_(new Impl())
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

    // The scene has no heading: a shuttle starts pointing along its velocity relative to its parent node (the same
    // parent a controller sees), not along the absolute velocity, which for a shuttle about a moving planet is the
    // planet's own motion.
    Hierarchy startHierarchy;
    if (!scene.shuttles.empty() && bodies_.size() > 0) {
        std::vector<Vec3> r(bodies_.size()), v(bodies_.size());
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            r[i] = bodyPos(bodies_, i);
            v[i] = {bodies_.state.vx[i], bodies_.state.vy[i], bodies_.state.vz[i]};
        }
        startHierarchy = buildHierarchy(r, v, bodies_.mu);
    }
    for (const ResolvedShuttle& r : scene.shuttles) {
        ShuttleDesc d;
        if (!shuttleDescFromResolved(r, d)) {
            warnings_.push_back("shuttle '" + r.desc.name + "' skipped: unknown controller '" + r.desc.controller + "'");
            continue;
        }
        if (startHierarchy.root >= 0) {
            const int parent = shuttleParent(startHierarchy, d.position);
            const Vec3 rel = d.velocity - startHierarchy.nodes[static_cast<std::size_t>(parent)].v;
            if (rel.x != 0.0 || rel.y != 0.0) d.heading = std::atan2(rel.y, rel.x);
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
                              {shuttles_.state.vx[i], shuttles_.state.vy[i], shuttles_.state.vz[i]}, shuttles_.heading[i],
                              shuttles_.spin[i]};
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

void Simulation::setControllerTick(double period)
{
    if (!(period > 0.0) || !std::isfinite(period)) return;
    controllerTick_ = period;
    envDirty_ = true;
    for (std::size_t i = 0; i < shuttles_.size(); ++i) shuttles_.nextTick[i] = kTickUnset;
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
    int stalled = 0;
    while (t_ < tTarget) {
        const double tBefore = t_;
        const std::size_t countBefore = bodies_.size();
        if (!stepBodies()) return false;
        // Every body step moves the clock or merges a pair. If neither happens repeatedly something is wrong in the contact
        // logic: stop with an error instead of spinning forever.
        if (t_ == tBefore && bodies_.size() == countBefore) {
            if (++stalled >= kMaxStalledSteps) {
                failed_ = true;
                warnings_.push_back("body step made no progress (no time advance, no merge)");
                return false;
            }
        } else {
            stalled = 0;
        }
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
    const auto wallStart = std::chrono::steady_clock::now();
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
    std::optional<std::pair<BodyId, BodyId>> forced; // the pair that clipped the step: merged at its end if it really touches
    // A contact within this many seconds of the step start counts as touching at the start: a clipped step that short
    // would not move the clock (4 ulp of t0), so the pair is merged instead of stepped to.
    const double touchEps = 4.0 * (std::nextafter(std::fabs(t0), INFINITY) - std::fabs(t0));
    for (int redo = 0; redo < kMaxBodyRedos && !contacts.empty() && contacts.front().t < (t - t0) * (1.0 - kEndContactFraction);
         ++redo) {
        const BodyContact first = contacts.front();
        const double tc = t0 + first.t;
        if (first.t <= touchEps || !(tc > t0)) {
            // Touching (or overlapping) at the start already: merge now without taking a step. The test and the merge
            // loop use the same threshold and the first pair is always merged, so time or the body count always changes.
            ias_ = backup_;
            if (!mergePair(first.a, first.b, t0)) return fail("body merge failed for a pair touching at the start of a step");
            for (const BodyContact& c : contacts) {
                if (c.t > touchEps) break;
                mergePair(c.a, c.b, t0); // a pair of an already merged body no longer resolves
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
    const auto wallShuttles = std::chrono::steady_clock::now();
    stats_.bodySeconds += std::chrono::duration<double>(wallShuttles - wallStart).count();
    advanceShuttles(s0, s1);

    // 4. Commit the bodies and merge the pairs that touch at the end of the step.
    bodies_.state = work_;
    t_ = t;
    // The pair that clipped the step is merged if it touches at the clipped end (the chord test can fire on chord sag
    // alone, for a near-grazing orbit that never touches; then the short step just ends and the next one tests again).
    // A pair that the retest still lists is merged by the loop below.
    if (forced && touchingNow(forced->first, forced->second)) mergePair(forced->first, forced->second, t_);
    for (const BodyContact& c : contacts) mergePair(c.a, c.b, t_); // a pair already merged above no longer resolves
    return true;
}

bool Simulation::touchingNow(BodyId a, BodyId b) const
{
    const std::optional<std::size_t> ia = bodies_.find(a), ib = bodies_.find(b);
    if (!ia || !ib) return false;
    const double R = bodies_.radius[*ia] + bodies_.radius[*ib];
    return norm(bodyPos(bodies_, *ia) - bodyPos(bodies_, *ib)) <= R * (1.0 + kTouchTolerance);
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
                                     {shuttles_.state.vx[i], shuttles_.state.vy[i], shuttles_.state.vz[i]},
                                     shuttles_.heading[i], shuttles_.spin[i]});
        }
    }
    if (n == 0 || shuttles_.aliveCount() == 0) return;

    const auto wallSetup = std::chrono::steady_clock::now();
    prepareEnvironment();
    Impl& I = *impl_;
    I.ephemeris.build(ias_);
    I.parents.build(bodies_, I.ephemeris, t0); // bodies_.state is still the state at the start of the step
    const ShuttleEnvironment& env = *I.env;

    const std::size_t tasks = std::min(n, kShuttleTasks);
    I.taskEvents.resize(tasks);
    I.taskStats.assign(tasks, ShuttleStats{});
    I.scratch.resize(tasks);
    for (auto& e : I.taskEvents) e.clear();

    const auto wallRun = std::chrono::steady_clock::now();
    stats_.shuttleSetupSeconds += std::chrono::duration<double>(wallRun - wallSetup).count();
    pool_.run(tasks, [&](std::size_t k) {
        const std::size_t lo = k * n / tasks, hi = (k + 1) * n / tasks;
        Impl::TaskScratch& sc = I.scratch[k];
        sc.work.positions.invalidate();   // a new body step: the dense output changed
        SweptHook hook(env, shuttles_, lastStep_, paths_, recordPaths_, sc.hits, sc.endPos, sc.prevEndPos, sc.work.positions);
        for (std::size_t i = lo; i < hi; ++i) {
            if (shuttles_.alive[i] == 0) continue;
            hook.beginShuttle();
            propagateShuttle(shuttles_, i, t0, t1, env, I.taskEvents[k], &hook, &I.taskStats[k], &sc.work);
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

    stats_.shuttleRunSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - wallRun).count();
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

double Simulation::shuttleHeadingAt(std::size_t i, double t) const
{
    if (recordPaths_ && !paths_[i].empty()) {
        const ShuttleSample *a, *b;
        bracket(paths_[i], t, a, b);
        return hermiteHeading(*a, *b, t);
    }
    return hermiteHeading(lastStep_[i].from, lastStep_[i].to, t);
}

double Simulation::shuttleSpinAt(std::size_t i, double t) const
{
    if (recordPaths_ && !paths_[i].empty()) {
        const ShuttleSample *a, *b;
        bracket(paths_[i], t, a, b);
        return hermiteSpin(*a, *b, t);
    }
    return hermiteSpin(lastStep_[i].from, lastStep_[i].to, t);
}

} // namespace cosmos
