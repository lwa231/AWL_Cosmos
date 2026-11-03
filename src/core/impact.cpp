#include "core/impact.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "core/collisions.h"
#include "core/kepler.h"

namespace cosmos {

namespace {
const double kPi = 3.141592653589793238462643383279502884;
const double kInf = std::numeric_limits<double>::infinity();

bool finite3(const Vec3& a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

// x^3/3! + s x^5/5! + s^2 x^7/7! + ...  with s = -1: x - sin x;  s = +1: sinh x - x. For |x| < 1 (ten terms are far
// below roundoff); keeps the small-anomaly case of the Kepler equation free of the cancellation in x - sin x.
double cubicSeries(double x, double s)
{
    const double x2 = x * x;
    double term = x * x2 / 6.0;
    double sum = term;
    for (int k = 2; k <= 12; ++k) {
        term *= s * x2 / ((2.0 * k) * (2.0 * k + 1.0));
        sum += term;
    }
    return sum;
}

double eMinusSin(double E) { return std::abs(E) < 1.0 ? cubicSeries(E, -1.0) : E - std::sin(E); }
double sinhMinus(double H) { return std::abs(H) < 1.0 ? cubicSeries(H, 1.0) : std::sinh(H) - H; }

// M = E - e sin E = (E - sin E) + (1 - e) sin E ; oneMinusE is supplied as (p/a)/(1+e), exact for e near 1.
double ellipticMean(double E, double oneMinusE) { return eMinusSin(E) + oneMinusE * std::sin(E); }

// M = e sinh H - H = (sinh H - H) + (e - 1) sinh H
double hyperbolicMean(double H, double eMinusOne) { return sinhMinus(H) + eMinusOne * std::sinh(H); }

ImpactPrediction invalid()
{
    ImpactPrediction o;
    o.kind = ImpactKind::Invalid;
    o.t = kInf;
    return o;
}
} // namespace

double triggerAtmosphereTop(const std::optional<Atmosphere>& atmosphere)
{
    if (!atmosphere) return 0.0;
    const Atmosphere& a = *atmosphere;
    if (!(a.rho0 > kAtmosphereCutoffDensity) || !(a.scaleHeight > 0.0)) return 0.0;
    return a.scaleHeight * std::log(a.rho0 / kAtmosphereCutoffDensity);
}

double triggerRadius(double radius, const std::optional<Atmosphere>& atmosphere)
{
    return radius + triggerAtmosphereTop(atmosphere);
}

ImpactPrediction predictParentImpact(const Vec3& r, const Vec3& v, double mu, double rTrig)
{
    if (!finite3(r) || !finite3(v) || !std::isfinite(mu) || !std::isfinite(rTrig) || !(mu > 0.0) || !(rTrig > 0.0))
        return invalid();
    const double rm = std::sqrt(r.x * r.x + r.y * r.y);
    if (!(rm > 0.0)) return invalid();

    const double v2 = v.x * v.x + v.y * v.y;
    const double rv = r.x * v.x + r.y * v.y;
    const double energy = 0.5 * v2 - mu / rm; // specific orbital energy
    const Elements el = stateToElements(r, v, mu);
    if (!std::isfinite(el.p) || !std::isfinite(el.e) || !std::isfinite(el.q)) return invalid();
    const bool bound = energy < 0.0;

    ImpactPrediction out;
    out.t = kInf;

    // Never leaves the sphere: drag handles it (plan 5.10). Checked first, so a bound orbit wholly inside is
    // reported as such rather than as "inside right now".
    if (bound && el.a * (1.0 + el.e) <= rTrig) {
        out.kind = ImpactKind::InsideBound;
        out.t = 0.0;
        return out;
    }
    if (rm <= rTrig) {
        out.kind = ImpactKind::Inside;
        out.t = 0.0;
        return out;
    }
    if (!(el.q < rTrig)) return out; // periapsis outside the trigger sphere: never reaches it
    // Unbound and not inbound (r.v >= 0): it only gets further away. A bound orbit that is outbound comes back round.
    const double rRef = std::max(rm, rTrig);
    const bool nearParabola = std::abs(energy) * rRef < 1e-15 * mu;
    if (!bound && !(rv < 0.0)) return out;
    if (bound && nearParabola && !(rv < 0.0)) return out; // period beyond ~1e22 dynamical times

    const double p = el.p;
    double t = 0.0;
    double eSinNu = 0.0; // e sin(nu_hit) (negative: inbound)

    if (nearParabola) {
        // Barker's equation in the q-safe form  t = sqrt(2/mu)/3 * [sqrt(r - q)(r + 2q)] between the two radii,
        // q = p/2 (also right for a radial parabola, q = 0).
        const double q = 0.5 * p;
        const double s0 = std::sqrt(rm - q) * (rm + 2.0 * q);
        const double s1 = std::sqrt(std::max(rTrig - q, 0.0)) * (rTrig + 2.0 * q);
        t = std::sqrt(2.0 / mu) / 3.0 * (s0 - s1);
        eSinNu = -2.0 * std::sqrt(std::max(q * (rTrig - q), 0.0)) / rTrig;
    } else if (bound) {
        const double a = el.a;
        const double pa = p / a;
        const double e = std::sqrt(std::max(1.0 - pa, 0.0));
        const double oneMinusE = pa / (1.0 + e);
        // e cos E = 1 - r/a, e sin E = r.v / sqrt(mu a)
        const double En = std::atan2(rv / std::sqrt(mu * a), 1.0 - rm / a);
        // At r = rTrig: e cos E = 1 - rTrig/a and (e sin E)^2 = e^2 - (e cos E)^2 = (2 rTrig - p)/a - (rTrig/a)^2
        const double xh = 1.0 - rTrig / a;
        const double y2 = (2.0 * rTrig - p) / a - (rTrig / a) * (rTrig / a);
        const double yh = -std::sqrt(std::max(y2, 0.0));
        const double Eh = std::atan2(yh, xh);
        double dM = ellipticMean(Eh, oneMinusE) - ellipticMean(En, oneMinusE);
        if (dM < 0.0) dM = (rv < 0.0) ? 0.0 : dM + 2.0 * kPi; // inbound: only roundoff can make it negative
        t = dM * std::sqrt(a * a * a / mu);
        eSinNu = yh * std::sqrt(a * p) / rTrig;
    } else {
        const double A = -el.a; // a < 0
        const double pA = p / A;
        const double e = std::sqrt(1.0 + pA);
        const double eMinusOne = pA / (1.0 + e);
        // e cosh H = 1 + r/A, e sinh H = r.v / sqrt(mu A)
        const double Hn = std::asinh(rv / std::sqrt(mu * A) / e);
        const double y2 = (2.0 * rTrig - p) / A + (rTrig / A) * (rTrig / A);
        const double yh = -std::sqrt(std::max(y2, 0.0));
        const double Hh = std::asinh(yh / e);
        const double dM = hyperbolicMean(Hh, eMinusOne) - hyperbolicMean(Hn, eMinusOne);
        t = std::max(dM, 0.0) * std::sqrt(A * A * A / mu);
        eSinNu = yh * std::sqrt(A * p) / rTrig;
    }

    // Crossing geometry. e cos(nu_hit) = p / rTrig - 1 (plan 5.10); the angle comes from atan2 of e sin and e cos,
    // which needs no division by e and stays exact for radial orbits (p = 0 gives nu = -pi).
    const double eCosNu = p / rTrig - 1.0;
    out.kind = ImpactKind::Impact;
    out.t = t;
    out.nuHit = std::atan2(eSinNu, eCosNu);
    out.speedHit = std::sqrt(std::max(v2 + 2.0 * mu * (1.0 / rTrig - 1.0 / rm), 0.0));
    // theta = omega + sign(h) nu (inverse of nu = sign(h) (theta - omega) in stateToElements)
    const double theta = el.omega + (el.retrograde ? -out.nuHit : out.nuHit);
    out.point = {rTrig * std::cos(theta), rTrig * std::sin(theta), 0.0};
    return out;
}

ImpactPrediction predictShuttleImpact(const Vec3& rShuttle, const Vec3& vShuttle, const Vec3& rParent, const Vec3& vParent,
                                      double muParent, double parentRadius, const std::optional<Atmosphere>& parentAtmosphere)
{
    return predictParentImpact(rShuttle - rParent, vShuttle - vParent, muParent, triggerRadius(parentRadius, parentAtmosphere));
}

ImpactPrediction predictPairImpact(const Vec3& pa, const Vec3& va, double muA, double radiusA,
                                   const Vec3& pb, const Vec3& vb, double muB, double radiusB)
{
    return predictParentImpact(pb - pa, vb - va, muA + muB, radiusA + radiusB);
}

ImpactPrediction predictPairImpact(const Bodies& bodies, std::size_t i, std::size_t j)
{
    const BodyState& s = bodies.state;
    const Vec3 pa{s.x[i], s.y[i], s.z[i]}, va{s.vx[i], s.vy[i], s.vz[i]};
    const Vec3 pb{s.x[j], s.y[j], s.z[j]}, vb{s.vx[j], s.vy[j], s.vz[j]};
    return predictPairImpact(pa, va, bodies.mu[i], bodies.radius[i], pb, vb, bodies.mu[j], bodies.radius[j]);
}

void predictPairImpacts(const Bodies& bodies, double horizon, std::vector<PairImpact>& out)
{
    out.clear();
    const std::size_t n = bodies.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const ImpactPrediction p = predictPairImpact(bodies, i, j);
            if (p.kind == ImpactKind::Impact && p.t <= horizon) out.push_back({i, j, bodies.ids[i], bodies.ids[j], p});
        }
    }
    std::sort(out.begin(), out.end(), [](const PairImpact& l, const PairImpact& r) {
        if (l.prediction.t != r.prediction.t) return l.prediction.t < r.prediction.t;
        if (l.a != r.a) return l.a < r.a;
        return l.b < r.b;
    });
}

std::optional<double> closingTime(double gap, double closingSpeed, double closingAccel)
{
    if (!std::isfinite(gap) || !std::isfinite(closingSpeed) || !std::isfinite(closingAccel)) return std::nullopt;
    if (!(gap > 0.0)) return 0.0;
    if (closingAccel == 0.0) {
        if (closingSpeed > 0.0) return gap / closingSpeed;
        return std::nullopt;
    }
    // (ac/2) t^2 + vc t - gap = 0
    const double disc = closingSpeed * closingSpeed + 2.0 * closingAccel * gap;
    if (!(disc >= 0.0)) return std::nullopt;
    const double q = -0.5 * (closingSpeed + std::copysign(std::sqrt(disc), closingSpeed));
    const double r1 = q / (0.5 * closingAccel);
    const double r2 = -gap / q;
    std::optional<double> best;
    for (double r : {r1, r2}) {
        if (std::isfinite(r) && r > 0.0 && (!best || r < *best)) best = r;
    }
    return best;
}

std::optional<double> estimateClosing(const Vec3& dp, const Vec3& dv, const Vec3& da, double contactDistance)
{
    const double d = norm(dp);
    const double gap = d - contactDistance;
    if (!(gap > 0.0)) return std::isfinite(gap) ? std::optional<double>(0.0) : std::nullopt;
    const double vc = -dot(dp, dv) / d;
    const double ac = -dot(dp, da) / d;
    return closingTime(gap, vc, ac);
}

double lookAheadTime(double warp) { return warp > 0.0 ? kLookAheadSeconds * warp : 0.0; }

std::optional<double> withinLookAhead(double t, double warp)
{
    if (!(t >= 0.0) || !std::isfinite(t)) return std::nullopt;
    if (t <= lookAheadTime(warp)) return t;
    return std::nullopt;
}

std::optional<double> impactWithinLookAhead(const ImpactPrediction& p, double warp, bool countInside)
{
    if (p.kind == ImpactKind::Impact) return withinLookAhead(p.t, warp);
    if (countInside && p.kind == ImpactKind::Inside) return 0.0;
    return std::nullopt;
}

SweptShuttleBody sweptShuttleVsBody(const Vec3& s0, const Vec3& s1, const Vec3& b0, const Vec3& b1, double dt,
                                    double bodyRadius, double shuttleRadius, double atmosphereTop)
{
    SweptShuttleBody out;
    const Vec3 p0 = s0 - b0;
    const Vec3 p1 = s1 - b1;

    const double rSurface = bodyRadius + shuttleRadius;
    out.surface = sweptContactTimeBetween(p0, p1, rSurface, dt);
    if (out.surface) {
        const double f = (dt > 0.0) ? std::min(*out.surface / dt, 1.0) : 0.0;
        out.surfacePoint = s0 + (s1 - s0) * f;
    }

    if (atmosphereTop > 0.0) {
        const double rAtm = std::max(bodyRadius + atmosphereTop, rSurface);
        if (norm2(p0) <= rAtm * rAtm) {
            out.startedInAtmosphere = true;
        } else {
            out.atmosphere = sweptContactTimeBetween(p0, p1, rAtm, dt);
        }
    }
    return out;
}

namespace {
double earliest(const SweptShuttleBody& c)
{
    double t = kInf;
    if (c.surface) t = std::min(t, *c.surface);
    if (c.atmosphere) t = std::min(t, *c.atmosphere);
    return t;
}
} // namespace

void findShuttleContacts(const Vec3& s0, const Vec3& s1, double shuttleRadius, const Bodies& bodies,
                         const BodyState& bodiesEnd, double dt, std::vector<ShuttleBodyHit>& out)
{
    out.clear();
    const BodyState& st = bodies.state;
    const std::size_t n = bodies.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 b0{st.x[i], st.y[i], st.z[i]};
        const Vec3 b1{bodiesEnd.x[i], bodiesEnd.y[i], bodiesEnd.z[i]};
        const SweptShuttleBody c = sweptShuttleVsBody(s0, s1, b0, b1, dt, bodies.radius[i], shuttleRadius,
                                                      triggerAtmosphereTop(bodies.atmosphere[i]));
        if (c.surface || c.atmosphere) out.push_back({i, bodies.ids[i], c});
    }
    std::sort(out.begin(), out.end(), [](const ShuttleBodyHit& l, const ShuttleBodyHit& r) {
        const double tl = earliest(l.contact), tr = earliest(r.contact);
        if (tl != tr) return tl < tr;
        return l.body < r.body;
    });
}

void appendShuttleStepEvents(std::vector<Event>& out, double tStart, BodyId shuttle, BodyId body, const SweptShuttleBody& c)
{
    if (c.atmosphere) out.push_back(makeAtmosphereEntry(tStart + *c.atmosphere, shuttle, body));
    if (c.surface) out.push_back(makeShuttleDestroyed(tStart + *c.surface, shuttle, body));
}

bool appendShuttleStepEvents(std::vector<Event>& out, double tStart, BodyId shuttle, const std::vector<ShuttleBodyHit>& hits)
{
    // The destroying contact: earliest surface time, ties by smaller body id.
    const ShuttleBodyHit* killer = nullptr;
    for (const ShuttleBodyHit& h : hits) {
        if (!h.contact.surface) continue;
        if (!killer || *h.contact.surface < *killer->contact.surface ||
            (*h.contact.surface == *killer->contact.surface && h.body < killer->body))
            killer = &h;
    }
    for (const ShuttleBodyHit& h : hits) {
        if (h.contact.atmosphere && (!killer || *h.contact.atmosphere <= *killer->contact.surface))
            out.push_back(makeAtmosphereEntry(tStart + *h.contact.atmosphere, shuttle, h.body));
    }
    if (killer) out.push_back(makeShuttleDestroyed(tStart + *killer->contact.surface, shuttle, killer->body));
    return killer != nullptr;
}

} // namespace cosmos
