#include "core/dopri5.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cosmos {

double dopri5VelocityScale(const Dopri5Tolerance& tol, double mu, double d)
{
    const double vc = d > 0.0 && mu > 0.0 ? std::sqrt(mu / d) : 0.0;
    return tol.atolVel + tol.rtol * vc;
}

double dopri5ErrorNorm(const Dopri5State& err, double posScale, double velScale)
{
    const double sp = 1.0 / posScale, sv = 1.0 / velScale;
    double sum = 0.0;
    const double comp[6] = {err.r.x * sp, err.r.y * sp, err.r.z * sp, err.v.x * sv, err.v.y * sv, err.v.z * sv};
    for (double q : comp) sum += q * q;
    const double n = std::sqrt(sum / 6.0);
    return std::isfinite(n) ? n : std::numeric_limits<double>::infinity();
}

double dopri5StepFactor(double err)
{
    if (!std::isfinite(err)) return 0.2;
    if (err <= 0.0) return 5.0;
    return std::min(5.0, std::max(0.2, 0.9 * std::pow(err, -0.2)));
}

} // namespace cosmos
