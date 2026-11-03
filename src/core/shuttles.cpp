#include "core/shuttles.h"
#include <algorithm>
#include <cmath>
#include "core/atmosphere.h"

namespace cosmos {

// ---------------------------------------------------------------------------------------------------------------
// Pure operators

BurnResult rocketBurn(double dryMass, double fuelMass, double throttle, double maxThrust, double exhaustVelocity,
                      double duration, bool infiniteFuel)
{
    BurnResult r;
    throttle = std::clamp(throttle, 0.0, 1.0);
    if (!(throttle > 0.0) || !(maxThrust > 0.0) || !(exhaustVelocity > 0.0) || !(duration > 0.0)) return r;
    const double thrust = throttle * maxThrust;
    const double m0 = dryMass + fuelMass;
    if (infiniteFuel) {
        r.burnTime = duration;
        r.dv = thrust * duration / m0;
        return r;
    }
    if (!(fuelMass > 0.0)) return r;
    const double mdot = thrust / exhaustVelocity;
    const double tEmpty = fuelMass / mdot;
    if (duration >= tEmpty) {
        r.emptied = true;
        r.burnTime = tEmpty;
        r.fuelUsed = fuelMass;
        r.dv = exhaustVelocity * std::log(m0 / dryMass);
    } else {
        r.burnTime = duration;
        r.fuelUsed = mdot * duration;
        r.dv = -exhaustVelocity * std::log1p(-r.fuelUsed / m0); // ve ln(m0 / (m0 - used))
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Store

bool shuttleDescFromResolved(const ResolvedShuttle& r, ShuttleDesc& out)
{
    ControllerKind kind;
    if (!parseControllerKind(r.desc.controller, kind)) return false;
    ShuttleDesc d;
    d.id = r.desc.id;
    d.name = r.desc.name;
    d.controller = kind;
    d.position = r.position;
    d.velocity = r.velocity;
    d.heading = (r.velocity.x != 0.0 || r.velocity.y != 0.0) ? std::atan2(r.velocity.y, r.velocity.x) : 0.0;
    d.spin = 0.0;
    d.dryMass = r.desc.dryMass;
    d.fuelMass = r.desc.fuelMass;
    d.maxThrust = r.desc.maxThrust;
    d.exhaustVelocity = r.desc.exhaustVelocity;
    d.cdA = r.desc.cdA;
    d.radius = r.desc.radius;
    d.maxTorque = r.desc.maxTorque;
    d.inertia = r.desc.inertia;
    out = d;
    return true;
}

namespace {
bool finite(double x) { return std::isfinite(x); }
} // namespace

bool Shuttles::add(const ShuttleDesc& d)
{
    if (d.id == 0 || used_.count(d.id) != 0) return false;
    if (!(d.dryMass > 0.0) || !finite(d.fuelMass) || d.fuelMass < 0.0) return false;
    for (double f : {d.maxThrust, d.exhaustVelocity, d.cdA, d.radius, d.maxTorque})
        if (!finite(f) || f < 0.0) return false;
    if (!finite(d.inertia) || !(d.inertia > 0.0)) return false;
    for (double f : {d.position.x, d.position.y, d.position.z, d.velocity.x, d.velocity.y, d.velocity.z, d.heading, d.spin})
        if (!finite(f)) return false;

    used_.insert(d.id);
    index_[d.id] = ids.size();
    ids.push_back(d.id);
    name.push_back(d.name);
    controller.push_back(d.controller);
    state.x.push_back(d.position.x);
    state.y.push_back(d.position.y);
    state.z.push_back(d.position.z);
    state.vx.push_back(d.velocity.x);
    state.vy.push_back(d.velocity.y);
    state.vz.push_back(d.velocity.z);
    heading.push_back(d.heading);
    spin.push_back(d.spin);
    dryMass.push_back(d.dryMass);
    fuelMass.push_back(d.fuelMass);
    maxThrust.push_back(d.maxThrust);
    exhaustVelocity.push_back(d.exhaustVelocity);
    cdA.push_back(d.cdA);
    radius.push_back(d.radius);
    maxTorque.push_back(d.maxTorque);
    inertia.push_back(d.inertia);
    throttle.push_back(0.0);
    turn.push_back(0.0);
    alive.push_back(1);
    fuelEmptyReported.push_back(0);
    nextTick.push_back(kTickUnset);
    nextStep.push_back(0.0);
    return true;
}

bool Shuttles::addResolved(const ResolvedShuttle& r)
{
    ShuttleDesc d;
    return shuttleDescFromResolved(r, d) && add(d);
}

std::optional<std::size_t> Shuttles::find(ShuttleId id) const
{
    const auto it = index_.find(id);
    if (it == index_.end()) return std::nullopt;
    return it->second;
}

std::size_t Shuttles::aliveCount() const
{
    std::size_t n = 0;
    for (std::uint8_t a : alive) n += a != 0 ? 1 : 0;
    return n;
}

void Shuttles::reindex()
{
    index_.clear();
    for (std::size_t i = 0; i < ids.size(); ++i) index_[ids[i]] = i;
}

void Shuttles::compact()
{
    std::size_t w = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (alive[i] == 0) continue;
        if (w != i) {
            ids[w] = ids[i];
            name[w] = std::move(name[i]);
            controller[w] = controller[i];
            state.x[w] = state.x[i]; state.y[w] = state.y[i]; state.z[w] = state.z[i];
            state.vx[w] = state.vx[i]; state.vy[w] = state.vy[i]; state.vz[w] = state.vz[i];
            heading[w] = heading[i]; spin[w] = spin[i];
            dryMass[w] = dryMass[i]; fuelMass[w] = fuelMass[i];
            maxThrust[w] = maxThrust[i]; exhaustVelocity[w] = exhaustVelocity[i]; cdA[w] = cdA[i];
            radius[w] = radius[i]; maxTorque[w] = maxTorque[i]; inertia[w] = inertia[i];
            throttle[w] = throttle[i]; turn[w] = turn[i];
            alive[w] = alive[i];
            fuelEmptyReported[w] = fuelEmptyReported[i];
            nextTick[w] = nextTick[i];
            nextStep[w] = nextStep[i];
        }
        ++w;
    }
    ids.resize(w); name.resize(w); controller.resize(w);
    state.x.resize(w); state.y.resize(w); state.z.resize(w);
    state.vx.resize(w); state.vy.resize(w); state.vz.resize(w);
    heading.resize(w); spin.resize(w); dryMass.resize(w); fuelMass.resize(w);
    maxThrust.resize(w); exhaustVelocity.resize(w); cdA.resize(w); radius.resize(w); maxTorque.resize(w); inertia.resize(w);
    throttle.resize(w); turn.resize(w); alive.resize(w); fuelEmptyReported.resize(w); nextTick.resize(w); nextStep.resize(w);
    reindex();
}

bool Shuttles::consistent() const
{
    const std::size_t n = ids.size();
    return name.size() == n && controller.size() == n && state.x.size() == n && state.y.size() == n && state.z.size() == n &&
           state.vx.size() == n && state.vy.size() == n && state.vz.size() == n && heading.size() == n && spin.size() == n &&
           dryMass.size() == n && fuelMass.size() == n && maxThrust.size() == n && exhaustVelocity.size() == n &&
           cdA.size() == n && radius.size() == n && maxTorque.size() == n && inertia.size() == n && throttle.size() == n &&
           turn.size() == n && alive.size() == n && fuelEmptyReported.size() == n && nextTick.size() == n &&
           nextStep.size() == n && index_.size() == n;
}

// ---------------------------------------------------------------------------------------------------------------
// Environment

ShuttleEnvironment::ShuttleEnvironment(const Bodies& b, const IBodyEphemeris& e, const ControllerSet& c)
    : bodies(b), ephemeris(e), controllers(c)
{
    atmosphereTop.resize(b.size(), 0.0);
    for (std::size_t j = 0; j < b.size(); ++j)
        if (b.atmosphere[j]) atmosphereTop[j] = cosmos::atmosphereTop(*b.atmosphere[j]);
}

// ---------------------------------------------------------------------------------------------------------------
// Propagation

namespace {

constexpr std::size_t kNone = static_cast<std::size_t>(-1);

// Shuttle state during a propagation; trial steps work on a copy.
struct Dyn {
    Vec3 r, v;
    double theta = 0.0;
    double omega = 0.0;
    double fuel = 0.0;
    bool reported = false;  // FuelEmpty already reported (committed)
    double emptyAt = std::numeric_limits<double>::quiet_NaN(); // FuelEmpty raised during this trial step
};

// Constants of one shuttle plus its held command.
struct Ctx {
    const ShuttleEnvironment& env;
    double dry, maxThrust, ve, cdA, maxTorque, inertia;
    bool infinite;
    double throttle, turn;

    double alpha() const { return maxTorque * turn / inertia; }
};

// Gravity a(t, r) = sum_j mu_j (r_j(t) - r) / |r_j(t) - r|^3 (plan 5.2) with the uniform-sphere interior field of plan 5.3
// for a shuttle inside a body (so that trial stages that dip into a body stay finite until the collision logic acts).
struct Gravity {
    const Bodies& b;
    const IBodyEphemeris& eph;
    mutable long calls = 0;

    Vec3 operator()(double t, const Vec3& r) const
    {
        ++calls;
        Vec3 a;
        const std::size_t n = b.size();
        for (std::size_t j = 0; j < n; ++j) {
            const Vec3 d = eph.position(j, t) - r;
            const double d2 = norm2(d);
            const double R = b.radius[j];
            if (d2 < R * R) {
                a += d * (b.mu[j] / (R * R * R));
            } else {
                const double inv = 1.0 / (d2 * std::sqrt(d2));
                a += d * (b.mu[j] * inv);
            }
        }
        return a;
    }
};

// The body that pulls hardest (largest mu / max(d, R)^2; ties go to the lowest index).
struct Dominant {
    std::size_t j = kNone;
    double d = 0.0;   // distance from the body's centre
    double mu = 0.0;
    Vec3 pos;
};

Dominant dominantBody(const ShuttleEnvironment& env, double t, const Vec3& r)
{
    Dominant best;
    double bestPull = -1.0;
    const Bodies& b = env.bodies;
    for (std::size_t j = 0; j < b.size(); ++j) {
        const Vec3 p = env.ephemeris.position(j, t);
        const double d2 = norm2(p - r);
        const double R = b.radius[j];
        const double pull = b.mu[j] / std::max(d2, R * R);
        if (pull > bestPull) {
            bestPull = pull;
            best.j = j;
            best.d = std::sqrt(d2);
            best.mu = b.mu[j];
            best.pos = p;
        }
    }
    return best;
}

double dynamicalTime(const Dominant& dom, const Bodies& b)
{
    if (dom.j == kNone || !(dom.mu > 0.0)) return 1.0;
    const double d = std::max({dom.d, b.radius[dom.j], 1.0});
    return std::sqrt(d * d * d / dom.mu);
}

// Step limit from the atmospheres (plan 5.6: drag is a split operator, so the step must resolve the density profile).
// Inside the reach (radius + top) of a body with an atmosphere the step is limited to min(activeTime, resolution * H / |v_rel|),
// that is a fraction of a scale height of travel. Outside, the step may be as long as the time to reach the atmosphere
// (but not shorter than the inside limit), so a fast shuttle cannot jump across the layer in one step. infinity = no limit.
double atmosphereStepCap(const ShuttleEnvironment& env, double activeTime, double t, const Vec3& r, const Vec3& v)
{
    const double inf = std::numeric_limits<double>::infinity();
    double cap = inf;
    const Bodies& b = env.bodies;
    for (std::size_t j = 0; j < b.size(); ++j) {
        const double top = env.atmosphereTop[j];
        if (!(top > 0.0)) continue;
        const Vec3 pos = env.ephemeris.position(j, t);
        const double dist = norm(r - pos);
        const double reach = b.radius[j] + top;
        const double vrel = std::max(norm(v - env.ephemeris.velocity(j, t)), 1.0);
        double inside = activeTime;
        if (env.atmosphereResolution > 0.0) inside = std::min(inside, env.atmosphereResolution * b.atmosphere[j]->scaleHeight / vrel);
        if (dist < reach) cap = std::min(cap, inside);
        else cap = std::min(cap, std::max((dist - reach) / vrel, inside));
    }
    return cap;
}

// ---- split operators on a trial state ----

void thrustOp(Dyn& z, const Ctx& c, double tSub, double d)
{
    if (!(c.throttle > 0.0) || !(d > 0.0)) return;
    if (!c.infinite && !(z.fuel > 0.0)) {
        // Out of fuel: throttle forced to 0 (plan 5.7); report once
        if (!z.reported && std::isnan(z.emptyAt)) z.emptyAt = tSub;
        return;
    }
    const BurnResult b = rocketBurn(c.dry, z.fuel, c.throttle, c.maxThrust, c.ve, d, c.infinite);
    if (!(b.dv > 0.0)) return;
    // Heading at the middle of the time the engine runs
    const double tm = 0.5 * b.burnTime;
    const Rotation mid = advanceRotation(z.theta, z.omega, c.alpha(), tm);
    z.v += Vec3{std::cos(mid.theta), std::sin(mid.theta), 0.0} * b.dv;
    if (!c.infinite) {
        z.fuel = b.emptied ? 0.0 : std::max(0.0, z.fuel - b.fuelUsed);
        if (b.emptied && !z.reported && std::isnan(z.emptyAt)) z.emptyAt = tSub + b.burnTime;
    }
}

void dragOp(Dyn& z, const Ctx& c, double tBody, double d)
{
    const Bodies& b = c.env.bodies;
    for (std::size_t j = 0; j < b.size(); ++j) {
        const double top = c.env.atmosphereTop[j];
        if (!(top > 0.0)) continue;
        const Vec3 pos = c.env.ephemeris.position(j, tBody);
        const double dist2 = norm2(z.r - pos);
        const double R = b.radius[j];
        const double reach = R + top;
        if (dist2 >= reach * reach) continue;
        const double h = std::sqrt(dist2) - R;
        const double rho = atmosphereDensity(*b.atmosphere[j], h);
        const double k = dragRate(rho, c.cdA, c.dry + z.fuel);
        const Vec3 vAir = airVelocity(c.env.ephemeris.velocity(j, tBody), b.spinRate[j], pos, z.r);
        z.v = applyExactDrag(z.v, vAir, k, d);
    }
}

void rotateOp(Dyn& z, const Ctx& c, double d)
{
    const Rotation rot = advanceRotation(z.theta, z.omega, c.alpha(), d);
    z.theta = rot.theta;
    z.omega = rot.omega;
}

// First half: thrust, drag, rotation. Second half: drag, thrust, rotation (the whole step is a palindrome).
void firstHalf(Dyn& z, const Ctx& c, double t, double h)
{
    const double d = 0.5 * h;
    thrustOp(z, c, t, d);
    dragOp(z, c, t, d);
    rotateOp(z, c, d);
}

void secondHalf(Dyn& z, const Ctx& c, double t, double h, double tEnd)
{
    const double d = 0.5 * h;
    dragOp(z, c, tEnd, d);
    thrustOp(z, c, t + d, d);
    rotateOp(z, c, d);
}

bool finiteState(const Dyn& z)
{
    return std::isfinite(z.r.x) && std::isfinite(z.r.y) && std::isfinite(z.r.z) && std::isfinite(z.v.x) &&
           std::isfinite(z.v.y) && std::isfinite(z.v.z) && std::isfinite(z.theta) && std::isfinite(z.omega);
}

double wrapHeading(double a) { return wrapAngle(a); }

} // namespace

void propagateShuttle(Shuttles& s, std::size_t i, double t0, double t1, const ShuttleEnvironment& env,
                      std::vector<Event>& events, IShuttleStepHook* hook, ShuttleStats* stats)
{
    if (!(t1 > t0) || s.alive[i] == 0) return;

    const Bodies& B = env.bodies;
    const IBodyEphemeris& eph = env.ephemeris;
    const double period = env.controllerTick;
    const ShuttleId id = s.ids[i];
    const Gravity grav{B, eph};

    Ctx c{env, s.dryMass[i], s.maxThrust[i], s.exhaustVelocity[i], s.cdA[i], s.maxTorque[i], s.inertia[i],
          s.infiniteFuel, s.throttle[i], s.turn[i]};

    Dyn y;
    y.r = {s.state.x[i], s.state.y[i], s.state.z[i]};
    y.v = {s.state.vx[i], s.state.vy[i], s.state.vz[i]};
    y.theta = s.heading[i];
    y.omega = s.spin[i];
    y.fuel = s.fuelMass[i];
    y.reported = s.fuelEmptyReported[i] != 0;

    auto tickTime = [&](std::int64_t k) { return static_cast<double>(k) * period; };
    std::int64_t nextTick = s.nextTick[i];
    double h = s.nextStep[i];
    double t = t0;
    bool killed = false;

    // Evaluate the shuttle's controller at tick k, time tNow
    auto evaluateTick = [&](std::int64_t k, double tNow) {
        ShuttleObservation obs;
        obs.shuttleId = id;
        obs.tick = k;
        obs.t_sim = tNow;
        obs.controllerTick_s = period;
        obs.position = y.r;
        obs.velocity = y.v;
        obs.heading = y.theta;
        obs.spin = y.omega;
        obs.mass = c.dry + y.fuel;
        obs.dryMass = c.dry;
        obs.fuelMass = y.fuel;
        obs.infiniteFuel = c.infinite;
        obs.maxThrust = c.maxThrust;
        obs.exhaustVelocity = c.ve;
        obs.maxTorque = c.maxTorque;
        obs.inertia = c.inertia;
        ParentRef parent;
        if (env.parents != nullptr) {
            parent = env.parents->parentOf(tNow, y.r);
        } else {
            const Dominant dom = dominantBody(env, tNow, y.r);
            if (dom.j != kNone) {
                parent.id = B.ids[dom.j];
                parent.mu = B.mu[dom.j];
                parent.r = dom.pos;
                parent.v = eph.velocity(dom.j, tNow);
            }
        }
        obs.parentId = parent.id;
        obs.parentMu = parent.mu;
        obs.relPosition = y.r - parent.r;
        obs.relVelocity = y.v - parent.v;
        obs.command = {c.throttle, c.turn};

        IController* ctrl = env.controllers.get(s.controller[i]);
        std::int64_t next = k + 1;
        if (ctrl != nullptr) {
            Rng rng = Rng::forTick(id, static_cast<std::uint64_t>(k));
            ShuttleCommand cmd = ctrl->update(obs, rng);
            c.throttle = std::isfinite(cmd.throttle) ? std::clamp(cmd.throttle, 0.0, 1.0) : 0.0;
            c.turn = std::isfinite(cmd.turn) ? std::clamp(cmd.turn, -1.0, 1.0) : 0.0;
            next = std::max(ctrl->nextTick(obs), k + 1);
        } else {
            c.throttle = 0.0;
            c.turn = 0.0;
        }
        nextTick = next;
        while (tickTime(nextTick) <= tNow) ++nextTick; // only ever loops on a rounding edge case
        if (stats) ++stats->controllerTicks;
    };

    // The tick whose time is the latest one not after t: tickTime(k) <= t < tickTime(k + 1), robust against rounding of t / period
    auto tickIndexAt = [&](double tNow) {
        std::int64_t k = static_cast<std::int64_t>(std::floor(tNow / period));
        while (tickTime(k + 1) <= tNow) ++k;
        while (tickTime(k) > tNow) --k;
        return k;
    };

    if (nextTick == kTickUnset) evaluateTick(tickIndexAt(t0), t0);

    Vec3 a0 = grav(t, y.r);

    while (t < t1) {
        // Controller tick due: the command may change here
        if (tickTime(nextTick) <= t) evaluateTick(std::max(nextTick, tickIndexAt(t)), t);

        const double limit = std::min(t1, tickTime(nextTick));

        const Dominant dom = dominantBody(env, t, y.r);
        if (!(h > 0.0)) h = 0.01 * dynamicalTime(dom, B);

        // Limits from the split operators (only while they are active)
        double cap = limit - t;
        const bool thrusting = c.throttle > 0.0 && (c.infinite || y.fuel > 0.0);
        {
            const double activeTime = env.activeStepFraction > 0.0 ? env.activeStepFraction * dynamicalTime(dom, B)
                                                                     : std::numeric_limits<double>::infinity();
            if (thrusting) cap = std::min(cap, activeTime);
            if (std::isfinite(activeTime) || env.atmosphereResolution > 0.0)
                cap = std::min(cap, atmosphereStepCap(env, activeTime, t, y.r, y.v));
        }
        if (thrusting && env.maxHeadingChangePerStep > 0.0) {
            // omega h + alpha h^2 / 2 <= limit angle (worst sign)
            const double w = std::fabs(y.omega), al = std::fabs(c.alpha()), lim = env.maxHeadingChangePerStep;
            double hRot = std::numeric_limits<double>::infinity();
            if (al > 0.0) hRot = (-w + std::sqrt(w * w + 2.0 * al * lim)) / al;
            else if (w > 0.0) hRot = lim / w;
            cap = std::min(cap, hRot);
        }
        const double hMin = std::max(env.minStep, 8.0 * (std::nextafter(std::fabs(t), std::numeric_limits<double>::infinity()) - std::fabs(t)));

        // Try steps until one is accepted
        Dyn z;
        Dopri5Result res;
        double hTry = 0.0, tEnd = t, err = 0.0;
        bool clipped = false;
        for (;;) {
            clipped = cap < h;
            hTry = clipped ? cap : h;
            tEnd = (hTry >= limit - t) ? limit : t + hTry;
            if (!(tEnd > t)) tEnd = limit; // h is below the resolution of t
            hTry = tEnd - t;

            z = y;
            z.emptyAt = std::numeric_limits<double>::quiet_NaN();
            firstHalf(z, c, t, hTry);
            res = dopri5Step(grav, t, hTry, Dopri5State{z.r, z.v}, a0);

            // Error norm with the scales of the dominant body's orbit (distance: the smaller of start and end)
            double d = std::max(norm(y.r), 1.0), mu = 0.0;
            if (dom.j != kNone) {
                const double d1 = norm(res.y1.r - eph.position(dom.j, tEnd));
                d = std::max({std::min(dom.d, d1), B.radius[dom.j], 1e-3});
                mu = dom.mu;
            }
            err = dopri5ErrorNorm(res.err, dopri5PositionScale(env.tolerance, d), dopri5VelocityScale(env.tolerance, mu, d));

            if (err <= 1.0 || hTry <= hMin) break;
            if (stats) ++stats->rejectedSteps;
            h = hTry * dopri5StepFactor(err);
        }

        // Accept
        z.r = res.y1.r;
        z.v = res.y1.v;
        secondHalf(z, c, t, hTry, tEnd);
        if (!finiteState(z)) {
            // Not committed: the shuttle keeps its last finite state
            events.push_back(makeShuttleDestroyed(t, id, 0));
            killed = true;
            break;
        }
        const double factor = dopri5StepFactor(err);
        h = clipped ? (factor >= 1.0 ? std::max(h, hTry * factor) : hTry * factor) : hTry * factor;
        if (stats) ++stats->acceptedSteps;

        if (!std::isnan(z.emptyAt)) {
            events.push_back(makeFuelEmpty(z.emptyAt, id));
            z.reported = true;
        }
        z.theta = wrapHeading(z.theta);

        ShuttleStepInfo info;
        info.id = id;
        info.index = i;
        info.t0 = t;
        info.t1 = tEnd;
        info.r0 = y.r;
        info.v0 = y.v;
        info.r1 = z.r;
        info.v1 = z.v;

        y = z;
        y.emptyAt = std::numeric_limits<double>::quiet_NaN();
        a0 = res.a1;
        t = tEnd;

        if (hook != nullptr && !hook->onAcceptedStep(info, events)) {
            killed = true;
            break;
        }
    }

    // Write back
    s.state.x[i] = y.r.x; s.state.y[i] = y.r.y; s.state.z[i] = y.r.z;
    s.state.vx[i] = y.v.x; s.state.vy[i] = y.v.y; s.state.vz[i] = y.v.z;
    s.heading[i] = y.theta;
    s.spin[i] = y.omega;
    s.fuelMass[i] = y.fuel;
    s.fuelEmptyReported[i] = y.reported ? 1 : 0;
    s.throttle[i] = c.throttle;
    s.turn[i] = c.turn;
    s.nextTick[i] = nextTick;
    s.nextStep[i] = h;
    if (killed) s.alive[i] = 0;
    if (stats) stats->forceCalls += grav.calls;
}

} // namespace cosmos
