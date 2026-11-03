#include "core/diagnostics.h"

namespace cosmos {

double pairPotentialG(double mui, double muj, double r, double rc)
{
    if (r < rc) return -(mui * muj) * (3.0 * rc * rc - r * r) / (2.0 * rc * rc * rc);
    return -(mui * muj / r);
}

namespace {
// Shared by conserved() and energyG(): adds the pair potentials onto e in a fixed (i, j>i) order,
// the same accumulation order as the original point-mass code.
void addPotentialsG(const Bodies& b, double& e)
{
    const BodyState& s = b.state;
    const std::size_t n = b.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Vec3 d{s.x[j] - s.x[i], s.y[j] - s.y[i], s.z[j] - s.z[i]};
            const double rc = b.radius[i] > b.radius[j] ? b.radius[i] : b.radius[j];
            e += pairPotentialG(b.mu[i], b.mu[j], norm(d), rc);
        }
    }
}
}

double energyG(const Bodies& b)
{
    const BodyState& s = b.state;
    double e = 0.0;
    for (std::size_t i = 0; i < b.size(); ++i) {
        const Vec3 v{s.vx[i], s.vy[i], s.vz[i]};
        e += 0.5 * b.mu[i] * norm2(v);
    }
    addPotentialsG(b, e);
    return e;
}

Conserved conserved(const Bodies& b)
{
    const BodyState& s = b.state;
    const std::size_t n = b.size();
    Conserved c;

    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 r{s.x[i], s.y[i], s.z[i]};
        const Vec3 v{s.vx[i], s.vy[i], s.vz[i]};

        c.energyG += 0.5 * b.mu[i] * norm2(v);
        c.momentumG += b.mu[i] * v;
        c.angularMomentumG += b.mu[i] * cross(r, v);
        c.angularMomentumG.z += b.inertiaFactor[i] * b.mu[i] * b.radius[i] * b.radius[i] * b.spinRate[i];
    }

    addPotentialsG(b, c.energyG);
    return c;
}

Conserved withMergesAddedBack(const Conserved& c, const MergeLedger& ledger)
{
    Conserved out = c;
    out.energyG -= ledger.energyG;
    out.angularMomentumG += ledger.lostAngularMomentumG;
    return out;
}

} // namespace cosmos
