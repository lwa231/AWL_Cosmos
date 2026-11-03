#include "core/atmosphere.h"
#include <cmath>

namespace cosmos {

double atmosphereDensity(const Atmosphere& a, double h)
{
    if (!(a.rho0 > 0.0) || !(a.scaleHeight > 0.0)) return 0.0;
    if (h <= 0.0) return a.rho0;
    return a.rho0 * std::exp(-h / a.scaleHeight);
}

double atmosphereTop(const Atmosphere& a)
{
    if (!(a.rho0 > kAtmosphereCutoffDensity) || !(a.scaleHeight > 0.0)) return 0.0;
    return a.scaleHeight * std::log(a.rho0 / kAtmosphereCutoffDensity);
}

double dragDensity(const Atmosphere& a, double h)
{
    if (h >= atmosphereTop(a)) return 0.0;
    return atmosphereDensity(a, h);
}

double surfaceGravity(double mu, double radius) { return mu / (radius * radius); }

double surfacePressure(const Atmosphere& a, double mu, double radius)
{
    return a.rho0 * surfaceGravity(mu, radius) * a.scaleHeight;
}

} // namespace cosmos
