#include "app/overlay_orbits.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

namespace {
const double kPi = 3.141592653589793238462643383279502884;
}

bool buildOrbitCurve(const Elements& el, double rCap, std::size_t segments, OrbitCurve& out)
{
    out.points.clear();
    out.hasApoapsis = false;
    out.closed = false;
    out.radiusMax = 0.0;
    if (segments < 8) segments = 8;
    if (!std::isfinite(el.p) || !std::isfinite(el.e) || !std::isfinite(el.omega) || !(el.p > 0.0) || !(el.e >= 0.0)) return false;

    const double sgn = el.h < 0.0 ? -1.0 : 1.0;
    double nuMax = kPi;
    if (el.e >= 1.0) {
        if (!(rCap > el.q)) return false;
        const double c = std::clamp((el.p / rCap - 1.0) / el.e, -1.0, 1.0);   // r(nu) = rCap
        nuMax = std::acos(c);
    }

    auto at = [&](double nu) {
        const double r = el.p / (1.0 + el.e * std::cos(nu));
        const double phi = el.omega + sgn * nu;
        return Vec3{r * std::cos(phi), r * std::sin(phi), 0.0};
    };

    out.points.reserve(segments + 1);
    for (std::size_t i = 0; i <= segments; ++i) {
        // nu = -nuMax + 2 nuMax i / n, with the middle point exactly 0 (periapsis) and the ends exactly +-nuMax.
        const double f = static_cast<double>(i) / static_cast<double>(segments);
        const double nu = i == segments ? nuMax : (i * 2 == segments ? 0.0 : -nuMax + 2.0 * nuMax * f);
        const Vec3 p = at(nu);
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            out.points.clear();
            return false;
        }
        out.points.push_back(p);
        out.radiusMax = std::max(out.radiusMax, std::hypot(p.x, p.y));
    }
    if (el.e < 1.0) out.points.back() = out.points.front();   // a closed loop without a seam
    out.closed = el.e < 1.0;
    out.periapsis = at(0.0);
    if (el.e < 1.0) {
        out.apoapsis = at(kPi);
        out.hasApoapsis = true;
    }
    return true;
}

} // namespace app
} // namespace cosmos
