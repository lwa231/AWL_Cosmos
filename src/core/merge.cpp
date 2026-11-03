#include "core/merge.h"
#include <cmath>
#include "core/constants.h"
#include "core/kinds.h"
#include "core/radii.h"

namespace cosmos {

namespace {
Vec3 posAt(const Bodies& b, std::size_t i) { return {b.state.x[i], b.state.y[i], b.state.z[i]}; }
Vec3 velAt(const Bodies& b, std::size_t i) { return {b.state.vx[i], b.state.vy[i], b.state.vz[i]}; }
}

std::optional<MergeResult> mergeBodies(Bodies& b, BodyId idA, BodyId idB, double t, EventQueue* events)
{
    if (idA == idB) return std::nullopt;
    const auto fa = b.find(idA);
    const auto fb = b.find(idB);
    if (!fa || !fb) return std::nullopt;
    const std::size_t ia = *fa;
    const std::size_t ib = *fb;

    // Survivor: heavier body, lower ID on a tie.
    const bool aSurvives = b.mu[ia] > b.mu[ib] || (b.mu[ia] == b.mu[ib] && idA < idB);
    const std::size_t is = aSurvives ? ia : ib; // survivor index
    const std::size_t io = aSurvives ? ib : ia; // absorbed index

    const double mu1 = b.mu[is];
    const double mu2 = b.mu[io];
    const Vec3 x1 = posAt(b, is), x2 = posAt(b, io);
    const Vec3 v1 = velAt(b, is), v2 = velAt(b, io);
    const double R1 = b.radius[is], R2 = b.radius[io];

    MergeResult m;
    m.survivor = b.ids[is];
    m.absorbed = b.ids[io];
    m.t = t;
    m.relSpeed = norm(v1 - v2);
    m.oldKind = b.kind[is];

    // Collision point on the line of centres, at the lower-ID body's surface (the midpoint for point masses).
    {
        const bool oneIsLower = b.ids[is] < b.ids[io];
        const Vec3 xLo = oneIsLower ? x1 : x2, xHi = oneIsLower ? x2 : x1;
        const double rLo = oneIsLower ? R1 : R2, rHi = oneIsLower ? R2 : R1;
        const double sum = rLo + rHi;
        const double f = sum > 0.0 ? rLo / sum : 0.5;
        m.point = xLo + (xHi - xLo) * f;
    }

    const double eBefore = energyG(b);

    // Conserved quantities (mass-weighted; G cancels because everything is mu).
    const double mu = mu1 + mu2;
    // Two massless bodies have no centre of mass; fall back to the plain mean instead of dividing 0 / 0.
    const Vec3 x = mu > 0.0 ? (mu1 * x1 + mu2 * x2) / mu : (x1 + x2) * 0.5;
    const Vec3 v = mu > 0.0 ? (mu1 * v1 + mu2 * v2) / mu : (v1 + v2) * 0.5;

    // S_G = spin1 + spin2 + sum mu_k (x_k - x) x (v_k - v). Stored spin is about z.
    const double spin1 = b.inertiaFactor[is] * mu1 * R1 * R1 * b.spinRate[is];
    const double spin2 = b.inertiaFactor[io] * mu2 * R2 * R2 * b.spinRate[io];
    const Vec3 orbital = mu1 * cross(x1 - x, v1 - v) + mu2 * cross(x2 - x, v2 - v);
    const double Sz = (spin1 + spin2) + orbital.z;

    m.newKind = mergedKind(b.kind[is], b.kindLocked[is] != 0, b.kind[io], mu);

    // Radius: volumes add; a Black Hole is exactly its Schwarzschild radius.
    const double R = m.newKind == BodyKind::BlackHole ? 2.0 * mu / (constants::c * constants::c)
                                                      : std::cbrt(R1 * R1 * R1 + R2 * R2 * R2);

    // Spin rate from S_G = I_G omega, clamped to the breakup rate.
    const double I = b.inertiaFactor[is] * mu * R * R;
    double omega = 0.0;
    Vec3 lost{orbital.x, orbital.y, 0.0}; // x/y orbital L cannot be held by a z-only spin
    if (I > 0.0) {
        m.omegaRaw = Sz / I;
        omega = m.omegaRaw;
        m.omegaBreak = breakupSpin(mu, R);
        if (std::abs(omega) > m.omegaBreak) {
            omega = std::copysign(m.omegaBreak, omega);
            m.spinClamped = true;
            lost.z = Sz - I * omega;
        }
    } else {
        lost.z = Sz; // point mass: no moment of inertia, no spin
    }
    m.lostAngularMomentumG = lost;

    // Write the survivor.
    b.mu[is] = mu;
    b.radius[is] = R;
    b.spinRate[is] = omega;
    b.state.x[is] = x.x; b.state.y[is] = x.y; b.state.z[is] = x.z;
    b.state.vx[is] = v.x; b.state.vy[is] = v.y; b.state.vz[is] = v.z;
    b.kind[is] = m.newKind;
    b.remove(m.absorbed);

    m.dEnergyG = energyG(b) - eBefore;

    if (events) {
        const BodyId lo = m.survivor < m.absorbed ? m.survivor : m.absorbed;
        const BodyId hi = m.survivor < m.absorbed ? m.absorbed : m.survivor;
        events->push(makeCollision(t, lo, hi, m.relSpeed, m.point));
        events->push(makeMerge(t, m.survivor, m.absorbed));
        if (m.newKind != m.oldKind) events->push(makeKindChanged(t, m.survivor, m.oldKind, m.newKind));
    }
    return m;
}

void record(MergeLedger& ledger, const MergeResult& m)
{
    ledger.energyG += m.dEnergyG;
    ledger.lostAngularMomentumG += m.lostAngularMomentumG;
}

} // namespace cosmos
