#pragma once
#include <cmath>
#include "core/constants.h"

namespace cosmos {

// All inputs in SI; masses enter only as mu (m^3/s^2), so G never appears except in meanDensity.

inline double circularSpeed(double muTotal, double r) { return std::sqrt(muTotal / r); }

// muM = orbiting body, muCentral = body it orbits
inline double hillRadius(double a, double e, double muM, double muCentral)
{
    return a * (1.0 - e) * std::cbrt(muM / (3.0 * muCentral));
}

inline double soiRadius(double a, double muM, double muCentral)
{
    return a * std::pow(muM / muCentral, 0.4);
}

// kg/m^3, uniform sphere (display and Roche only)
inline double meanDensity(double mu, double radius)
{
    const double pi = 3.141592653589793238462643383279502884;
    return mu / (constants::G * (4.0 / 3.0) * pi * radius * radius * radius);
}

// radiusPrimary and densityPrimary belong to the body being orbited; densityMoon to the satellite
inline double rocheLimit(double radiusPrimary, double densityPrimary, double densityMoon)
{
    return 2.44 * radiusPrimary * std::cbrt(densityPrimary / densityMoon);
}

inline double mutualHillRadius(double mu1, double mu2, double muCentral, double a1, double a2)
{
    return std::cbrt((mu1 + mu2) / (3.0 * muCentral)) * 0.5 * (a1 + a2);
}

inline double breakupSpin(double mu, double radius) { return std::sqrt(mu / (radius * radius * radius)); }

} // namespace cosmos
