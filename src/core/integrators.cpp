#include "core/integrators.h"

namespace cosmos {

void stepKDK(BodyState& s, const std::vector<double>& mu, Accel& a, double dt)
{
    const std::size_t n = s.x.size();
    const double half = 0.5 * dt;

    for (std::size_t i = 0; i < n; ++i) {
        s.vx[i] += a.ax[i] * half;
        s.vy[i] += a.ay[i] * half;
        s.vz[i] += a.az[i] * half;
    }
    for (std::size_t i = 0; i < n; ++i) {
        s.x[i] += s.vx[i] * dt;
        s.y[i] += s.vy[i] * dt;
        s.z[i] += s.vz[i] * dt;
    }

    computeAccelerations(s, mu, a);

    for (std::size_t i = 0; i < n; ++i) {
        s.vx[i] += a.ax[i] * half;
        s.vy[i] += a.ay[i] * half;
        s.vz[i] += a.az[i] * half;
    }
}

} // namespace cosmos
