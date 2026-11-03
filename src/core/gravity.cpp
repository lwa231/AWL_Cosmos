#include "core/gravity.h"
#include <cmath>

namespace cosmos {

void computeAccelerations(const BodyState& s, const std::vector<double>& mu, Accel& a)
{
    const std::size_t n = s.x.size();
    a.ax.assign(n, 0.0);
    a.ay.assign(n, 0.0);
    a.az.assign(n, 0.0);

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = s.x[j] - s.x[i];
            const double dy = s.y[j] - s.y[i];
            const double dz = s.z[j] - s.z[i];
            const double d2 = (dx * dx + dy * dy) + dz * dz;
            const double inv = 1.0 / (d2 * std::sqrt(d2));

            a.ax[i] += mu[j] * dx * inv;
            a.ay[i] += mu[j] * dy * inv;
            a.az[i] += mu[j] * dz * inv;
            a.ax[j] -= mu[i] * dx * inv;
            a.ay[j] -= mu[i] * dy * inv;
            a.az[j] -= mu[i] * dz * inv;
        }
    }
}

// Plan section 5.3. Identical to the point-mass loop except for the choice of `inv`: with R_c = 0 the test
// d2 < R_c^2 is never true, so the else branch runs the same instructions as above.
void computeAccelerations(const BodyState& s, const std::vector<double>& mu, const std::vector<double>& radius, Accel& a)
{
    const std::size_t n = s.x.size();
    a.ax.assign(n, 0.0);
    a.ay.assign(n, 0.0);
    a.az.assign(n, 0.0);

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = s.x[j] - s.x[i];
            const double dy = s.y[j] - s.y[i];
            const double dz = s.z[j] - s.z[i];
            const double d2 = (dx * dx + dy * dy) + dz * dz;
            const double rc = radius[i] > radius[j] ? radius[i] : radius[j];

            double inv;
            if (d2 < rc * rc) inv = 1.0 / (rc * rc * rc); // inside: a = mu d / R_c^3
            else inv = 1.0 / (d2 * std::sqrt(d2));

            a.ax[i] += mu[j] * dx * inv;
            a.ay[i] += mu[j] * dy * inv;
            a.az[i] += mu[j] * dz * inv;
            a.ax[j] -= mu[i] * dx * inv;
            a.ay[j] -= mu[i] * dy * inv;
            a.az[j] -= mu[i] * dz * inv;
        }
    }
}

} // namespace cosmos
