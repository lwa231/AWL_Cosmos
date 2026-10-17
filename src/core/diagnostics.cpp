#include "core/diagnostics.h"

namespace cosmos {

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

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Vec3 d{s.x[j] - s.x[i], s.y[j] - s.y[i], s.z[j] - s.z[i]};
            c.energyG -= b.mu[i] * b.mu[j] / norm(d);
        }
    }
    return c;
}

} // namespace cosmos
