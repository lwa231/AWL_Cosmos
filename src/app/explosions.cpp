#include "app/explosions.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "core/rng.h"

namespace cosmos {
namespace app {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNearbyMinPull = 0.01;   // a body joins the list when its pull at the anchor is at least this fraction of the main body's

std::uint64_t bitsOf(double d)
{
    std::uint64_t u = 0;
    std::memcpy(&u, &d, sizeof u);
    return u;
}

// Remove [a, b) from [s, e); up to two remnants are appended to out.
void subtractRange(const ExplosionSystem::Range& r, std::uint32_t a, std::uint32_t b, std::vector<ExplosionSystem::Range>& out)
{
    const std::uint32_t s = r.first, e = r.first + r.count;
    if (b <= s || a >= e) {
        out.push_back(r);
        return;
    }
    if (a > s) out.push_back({s, a - s});
    if (b < e) out.push_back({b, e - b});
}

Vec3 unitOrX(const Vec3& v)
{
    const double n = std::sqrt(v.x * v.x + v.y * v.y);
    return n > 0.0 ? Vec3{v.x / n, v.y / n, 0.0} : Vec3{1.0, 0.0, 0.0};
}

} // namespace

ExplosionParams explosionParams(double impactorRadius, double impactSpeed)
{
    // Scaling (plan 8 item 3, "scaled"): a 2 m shuttle gives the base cloud; each decade of impactor radius adds fragments, speed
    // range and lifetime, up to a 12k-fragment cloud for a planet-sized body. The ejecta speed is a small fraction of the impact
    // speed plus a size term; slow impacts still burst a little.
    const double r = std::max(impactorRadius, 0.1);
    const double speed = std::isfinite(impactSpeed) ? std::max(impactSpeed, 0.0) : 0.0;
    const double decades = std::log10(std::max(r / 2.0, 1.0));
    ExplosionParams p;
    p.fragments = static_cast<std::uint32_t>(std::min(12000.0, 2500.0 + 2400.0 * decades));
    p.speedMax = 20.0 + 0.05 * speed + 0.002 * r;
    p.speedMin = 0.15 * p.speedMax;
    p.lifeMax = 20.0 + 8.0 * decades;
    p.lifeMin = 0.4 * p.lifeMax;
    p.ballisticMin = 0.003;
    p.ballisticMax = 0.05;
    p.halfAngle = 80.0 * kPi / 180.0;
    p.spawnRadius = std::min(std::max(r, 1.0), 2.0e5);
    return p;
}

ExplosionSystem::ExplosionSystem(const ExplosionConfig& config) : config_(config)
{
    config_.maxFragments = std::max<std::uint32_t>(1, std::min(config_.maxFragments, kExplosionMaxFragments));
}

void ExplosionSystem::clear()
{
    explosions_.clear();
    pending_.clear();
    seen_.clear();
    head_ = 0;
    highWater_ = 0;
}

std::uint32_t ExplosionSystem::liveFragments() const
{
    std::uint32_t n = 0;
    for (const Explosion& e : explosions_) n += e.live;
    return n;
}

std::vector<ExplosionSystem::Range> ExplosionSystem::allocate(std::uint32_t n)
{
    const std::uint32_t pool = config_.maxFragments;
    std::vector<Range> pieces;
    const std::uint32_t c1 = std::min(n, pool - head_);
    pieces.push_back({head_, c1});
    if (n > c1) pieces.push_back({0, n - c1});

    // Recycling oldest-first: the slots a new explosion takes belong to the oldest data on the ring
    for (Explosion& e : explosions_) {
        for (const Range& piece : pieces) {
            std::vector<Range> kept;
            for (const Range& r : e.ranges) subtractRange(r, piece.first, piece.first + piece.count, kept);
            e.ranges.swap(kept);
        }
        e.live = 0;
        for (const Range& r : e.ranges) e.live += r.count;
    }
    explosions_.erase(std::remove_if(explosions_.begin(), explosions_.end(), [](const Explosion& e) { return e.live == 0; }),
                      explosions_.end());

    head_ = (head_ + n) % pool;
    for (const Range& piece : pieces) highWater_ = std::max(highWater_, piece.first + piece.count);
    return pieces;
}

std::uint32_t ExplosionSystem::freeSlot()
{
    if (explosions_.size() >= kExplosionMaxLive) explosions_.pop_front();   // the oldest explosion makes room (its fragments die)
    std::vector<bool> used(kExplosionMaxLive, false);
    for (const Explosion& e : explosions_) used[e.slot] = true;
    for (std::uint32_t s = 0; s < kExplosionMaxLive; ++s)
        if (!used[s]) return s;
    return 0;
}

std::uint32_t ExplosionSystem::spawn(const ExplosionSpec& spec)
{
    const ExplosionParams p = explosionParams(spec.impactorRadius, std::sqrt(spec.impactVelocity.x * spec.impactVelocity.x +
                                                                               spec.impactVelocity.y * spec.impactVelocity.y));
    const std::uint32_t n = std::min(p.fragments, config_.maxFragments);
    if (n == 0) return 0;

    const std::vector<Range> ranges = allocate(n);
    Explosion ex;
    ex.slot = freeSlot();
    ex.serial = ++serialCounter_;
    ex.refBody = spec.refBody;
    ex.offset = spec.contact - spec.refPos;
    ex.anchor = spec.contact;
    ex.lastRefVel = spec.refVel;
    ex.lifeMax = p.lifeMax;
    ex.minRealMax = config_.minRealVisible * 1.4;
    ex.ranges = ranges;
    ex.live = n;

    // Visual random stream: keyed by the source and the event time, independent of every physics stream (plan 4.6).
    Rng rng = Rng::forTick(spec.sourceId ^ config_.seed, bitsOf(spec.time));
    const Vec3 nrm = unitOrX(spec.normal);
    const Vec3 tan = {-nrm.y, nrm.x, 0.0};
    const Vec3 d = spec.contact - spec.refPos;
    const Vec3 ground = {-spec.refSpin * d.y, spec.refSpin * d.x, 0.0};   // the surface moves with the spin
    const double vn = spec.impactVelocity.x * nrm.x + spec.impactVelocity.y * nrm.y;
    const Vec3 skid = {spec.impactVelocity.x - vn * nrm.x, spec.impactVelocity.y - vn * nrm.y, 0.0};   // tangential part of the impact

    std::vector<FragmentGpu> frags(n);
    for (FragmentGpu& f : frags) {
        const double a = p.halfAngle * (rng.uniform() < 0.5 ? -1.0 : 1.0) * std::pow(rng.uniform(), 1.3);
        const double ca = std::cos(a), sa = std::sin(a);
        const Vec3 dir = {nrm.x * ca - nrm.y * sa, nrm.x * sa + nrm.y * ca, 0.0};
        const double u = rng.uniform();
        const double speed = p.speedMin + (p.speedMax - p.speedMin) * u * u;
        const double skidK = 0.3 * rng.uniform();
        const double up = 0.4 + 2.5 * rng.uniform();                       // start just above the contact point
        const double side = p.spawnRadius * (2.0 * rng.uniform() - 1.0) * 0.5;
        f.px = static_cast<float>(nrm.x * up + tan.x * side);
        f.py = static_cast<float>(nrm.y * up + tan.y * side);
        f.vx = static_cast<float>(ground.x + dir.x * speed + skid.x * skidK);
        f.vy = static_cast<float>(ground.y + dir.y * speed + skid.y * skidK);
        const double q = rng.uniform();
        f.life = static_cast<float>(p.lifeMin + (p.lifeMax - p.lifeMin) * q);
        f.minReal = static_cast<float>(config_.minRealVisible * (1.0 + 0.4 * rng.uniform()));
        const double w = rng.uniform();
        f.ballistic = static_cast<float>(p.ballisticMin + (p.ballisticMax - p.ballisticMin) * w * w);
        f.size = static_cast<float>(0.7 + 0.6 * rng.uniform());
        f.slot = ex.slot;
        f.serial = ex.serial;
        f.seed = static_cast<std::uint32_t>(rng.next());
    }
    std::uint32_t at = 0;
    for (const Range& r : ranges) {
        FragmentBatch b;
        b.firstSlot = r.first;
        b.fragments.assign(frags.begin() + at, frags.begin() + at + r.count);
        at += r.count;
        pending_.push_back(std::move(b));
    }
    explosions_.push_back(ex);
    return ex.serial;
}

bool ExplosionSystem::bodyAt(std::uint64_t id, double t, const DisplayState& st, const Simulation& sim, Vec3& r, Vec3& v) const
{
    // The dense output of the latest body step is exact, but only inside that step: Simulation clamps t to the step, and at
    // high warp a frame spans several body steps, so an event time can lie before the latest step.
    if (sim.hasBodyStep() && t >= sim.bodyStepStart() && t <= sim.bodyStepEnd() && sim.bodyPositionAt(id, t, r) &&
        sim.bodyVelocityAt(id, t, v))
        return true;
    const auto prev = seen_.find(id);
    for (const DisplayBody& b : st.bodies)
        if (b.id == id && !b.absorbed) {
            // t lies between the previous frame's sample and this display state: a cubic Hermite through both (error ~ a*dt^2)
            // beats extrapolating back from one end. Without a usable earlier sample, extrapolate along the velocity.
            if (prev != seen_.end() && prev->second.t < t && t <= st.t && st.t > prev->second.t) {
                const Seen& p = prev->second;
                const double h = st.t - p.t, s = (t - p.t) / h, s2 = s * s, s3 = s2 * s;
                const double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
                const double d00 = (6 * s2 - 6 * s) / h, d10 = (3 * s2 - 4 * s + 1), d01 = (-6 * s2 + 6 * s) / h, d11 = (3 * s2 - 2 * s);
                r = p.r * h00 + p.v * (h10 * h) + b.r * h01 + b.v * (h11 * h);
                v = p.r * d00 + p.v * d10 + b.r * d01 + b.v * d11;
                return true;
            }
            r = b.r - b.v * (st.t - t);   // back from the display time along the velocity
            v = b.v;
            return true;
        }
    const auto it = seen_.find(id);
    if (it != seen_.end()) {
        r = it->second.r;
        v = it->second.v;
        return true;
    }
    return false;
}

void ExplosionSystem::handleEvents(const std::vector<Event>& due, const DisplayState& st, const Simulation& sim)
{
    for (const Event& e : due) {
        ExplosionSpec spec;
        spec.time = e.t;
        if (e.type == EventType::ShuttleDestroyed) {
            if (e.other == 0) continue;   // not destroyed by a body (a non-finite state): nothing to show
            const Shuttles& sh = sim.shuttles();
            const auto idx = sh.find(e.id);
            if (!idx) continue;
            const std::size_t i = *idx;
            // A dead shuttle's store state is its contact point
            spec.contact = {sh.state.x[i], sh.state.y[i], 0.0};
            const Vec3 vS = {sh.state.vx[i], sh.state.vy[i], 0.0};
            Vec3 rb, vb;
            if (!bodyAt(e.other, e.t, st, sim, rb, vb)) continue;
            spec.refBody = e.other;
            spec.refPos = rb;
            spec.refVel = vb;
            spec.impactVelocity = vS - vb;
            spec.impactorRadius = sh.radius[i];
            spec.sourceId = e.id;
            spec.normal = unitOrX(spec.contact - rb);
        } else if (e.type == EventType::Collision) {
            // The survivor of the merge that goes with this collision is the body the cloud stays with
            std::uint64_t survivor = 0;
            for (const Event& m : due)
                if (m.type == EventType::Merge && m.t == e.t &&
                    ((m.id == e.id && m.other == e.other) || (m.id == e.other && m.other == e.id)))
                    survivor = m.id;
            const auto sa = seen_.find(e.id), sb = seen_.find(e.other);
            if (survivor == 0) {   // no merge listed: the heavier body
                const double ma = sa != seen_.end() ? sa->second.mu : 0.0, mb = sb != seen_.end() ? sb->second.mu : 0.0;
                survivor = mb > ma ? e.other : e.id;
            }
            const std::uint64_t absorbed = survivor == e.id ? e.other : e.id;
            Vec3 rs, vs;
            if (!bodyAt(survivor, e.t, st, sim, rs, vs)) continue;
            spec.contact = e.point;
            spec.refBody = survivor;
            spec.refPos = rs;
            spec.refVel = vs;
            spec.normal = unitOrX(e.point - rs);
            spec.impactVelocity = spec.normal * (-e.relSpeed);   // fallback: head-on at the recorded speed
            double radius = 0.0;
            const auto ia = seen_.find(absorbed), is = seen_.find(survivor);
            if (ia != seen_.end()) {
                spec.impactVelocity = ia->second.v - vs;
                radius = ia->second.radius;
            }
            if (is != seen_.end()) {
                spec.refSpin = is->second.spin;
                radius = radius > 0.0 ? std::min(radius, is->second.radius) : is->second.radius;
            }
            spec.impactorRadius = radius > 0.0 ? radius : 1.0e3;
            spec.sourceId = e.id ^ (e.other << 1);
        } else {
            continue;
        }
        if (spec.refBody != 0 && spec.refSpin == 0.0) {
            const auto it = seen_.find(spec.refBody);
            if (it != seen_.end()) spec.refSpin = it->second.spin;
            for (const DisplayBody& b : st.bodies)
                if (b.id == spec.refBody && !b.absorbed) spec.refSpin = b.spinRate;
        }
        spawn(spec);
    }
}

void ExplosionSystem::update(const DisplayState& st, double simDt, double realDt, const ToCamera& toCamera, ExplosionFrame& out)
{
    out.clear();
    for (const DisplayBody& b : st.bodies) {
        Seen& s = seen_[b.id];
        s.r = b.r;
        s.v = b.v;
        s.mu = b.mu;
        s.radius = b.radius;
        s.spin = b.spinRate;
        s.t = st.t;
    }

    const bool running = simDt > 0.0;
    const double realStep = std::min(std::max(realDt, 0.0), config_.maxRealDt);
    std::vector<const DisplayBody*> bodies;
    for (const DisplayBody& b : st.bodies)
        if (!b.absorbed) bodies.push_back(&b);

    out.table.assign(kExplosionMaxLive, ExplosionGpu{});
    bool any = false;
    for (auto it = explosions_.begin(); it != explosions_.end();) {
        Explosion& ex = *it;
        const bool fresh = ex.fresh;
        if (!fresh && running) {
            ex.simAge += simDt;
            ex.realAge += realStep;
        }
        if (ex.simAge >= ex.lifeMax && ex.realAge >= ex.minRealMax) {
            it = explosions_.erase(it);
            continue;
        }

        // The anchor travels with its body; without it, with the body's last velocity
        const DisplayBody* ref = nullptr;
        for (const DisplayBody* b : bodies)
            if (b->id == ex.refBody) ref = b;
        if (ref) {
            ex.anchor = ref->r + ex.offset;
            ex.lastRefVel = ref->v;
        } else if (!fresh) {
            ex.anchor += ex.lastRefVel * (running ? simDt : 0.0);
        }

        ExplosionGpu& g = out.table[ex.slot];
        g.serial = ex.serial;
        toCamera(ex.anchor, g.anchorX, g.anchorY);
        g.dtSim = fresh || !running ? 0.0f : static_cast<float>(simDt);
        g.dtReal = fresh || !running ? 0.0f : static_cast<float>(realStep);

        // Bodies acting on the cloud: the anchor's body first, then the strongest pulls
        struct Cand { const DisplayBody* b; double score; };
        std::vector<Cand> cands;
        double refScore = 0.0;
        for (const DisplayBody* b : bodies) {
            const Vec3 c = b->r - ex.anchor;
            const double d2 = std::max(c.x * c.x + c.y * c.y, b->radius * b->radius);
            const double score = d2 > 0.0 ? b->mu / d2 : 0.0;
            if (b == ref) refScore = score;
            cands.push_back({b, score});
        }
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.score > b.score; });
        std::vector<const DisplayBody*> chosen;
        if (ref) chosen.push_back(ref);
        const double floorScore = ref ? kNearbyMinPull * refScore : 0.0;
        for (const Cand& c : cands) {
            if (chosen.size() >= kExplosionMaxBodies) break;
            if (c.b == ref || c.score <= 0.0 || c.score < floorScore) continue;
            chosen.push_back(c.b);
        }
        g.bodyCount = static_cast<std::uint32_t>(chosen.size());
        g.ground = ref ? 1u : 0u;   // chosen[0] is the anchor's own body
        for (std::size_t k = 0; k < chosen.size(); ++k) {
            const DisplayBody& b = *chosen[k];
            const Vec3 c = b.r - ex.anchor;
            ExplosionBodyGpu& o = g.bodies[k];
            o.x = static_cast<float>(c.x);
            o.y = static_cast<float>(c.y);
            o.mu = static_cast<float>(b.mu);
            o.radius = static_cast<float>(b.radius);
            if (ref && &b != ref) {   // tidal term: how much this body accelerates the anchor's own body
                const Vec3 s = b.r - ref->r;
                const double d = std::max(std::sqrt(s.x * s.x + s.y * s.y), b.radius);
                const double k3 = b.mu / (d * d * d);
                o.indirectX = static_cast<float>(s.x * k3);
                o.indirectY = static_cast<float>(s.y * k3);
                o.vx = static_cast<float>(b.v.x - ref->v.x);
                o.vy = static_cast<float>(b.v.y - ref->v.y);
            }
            if (b.atmosphere && b.atmosphere->rho0 > 0.0 && b.atmosphere->scaleHeight > 0.0) {
                o.rho0 = static_cast<float>(b.atmosphere->rho0);
                o.scaleHeight = static_cast<float>(b.atmosphere->scaleHeight);
            }
            o.spin = static_cast<float>(b.spinRate);
            o.altitude = static_cast<float>(std::sqrt(c.x * c.x + c.y * c.y) - b.radius);   // double precision here, then rounded
        }
        ex.fresh = false;
        any = true;
        ++it;
    }

    if (!any) {
        out.table.clear();
        out.spawns.clear();
        pending_.clear();   // the spawns belonged to explosions that are already over
    } else {
        out.spawns.swap(pending_);
        pending_.clear();
    }
    out.fragmentCount = highWater_;
}

} // namespace app
} // namespace cosmos
