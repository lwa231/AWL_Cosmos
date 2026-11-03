#include "core/integrators.h"
#include <cmath>

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

void stepYoshida4(BodyState& s, const std::vector<double>& mu, Accel& a, double dt)
{
    const double c  = std::cbrt(2.0);
    const double w1 = 1.0 / (2.0 - c);       //  1.3512071919596578
    const double w0 = -c / (2.0 - c);        // -1.7024143839193153  (2*w1 + w0 = 1, 2*w1^3 + w0^3 = 0)

    // stepKDK leaves a valid at the new positions, so each sub-step reuses the previous one's accelerations
    stepKDK(s, mu, a, w1 * dt);
    stepKDK(s, mu, a, w0 * dt);
    stepKDK(s, mu, a, w1 * dt);
}

} // namespace cosmos
