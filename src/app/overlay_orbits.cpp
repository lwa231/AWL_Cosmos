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

    // The polyline is symmetric about the periapsis axis (nu -> -nu gives the same r), so one cos and one sin of nu serve a
    // pair of points, and the rotation by omega is the angle-addition formula: a point costs half a transcendental call instead
    // of three (the orbits were a large share of the line-geometry cost of a frame with 200 shuttles). Results agree with the
    // direct formula to rounding.
    const double cw = std::cos(el.omega), sw = std::sin(el.omega);
    out.points.resize(segments + 1);
    auto put = [&](std::size_t i, double r, double cn, double sn) {
        // phi = omega + sgn * nu: cos phi = cw cn - sgn sw sn, sin phi = sw cn + sgn cw sn
        out.points[i] = Vec3{r * (cw * cn - sgn * sw * sn), r * (sw * cn + sgn * cw * sn), 0.0};
    };
    // cos and sin of nu advance by a rotation of the constant step between exact evaluations every kResync points (the rotation
    // error grows by about 1e-16 a step, so it stays below 2e-15 and the polyline is the same to well under a pixel at any zoom).
    constexpr std::size_t kResync = 16;
    const double dnu = 2.0 * nuMax / static_cast<double>(segments);
    const double cd = std::cos(dnu), sd = std::sin(dnu);
    double cn = 1.0, sn = 0.0;
    for (std::size_t i = 0; i * 2 <= segments; ++i) {
        // nu = -nuMax + 2 nuMax i / n, with the middle point exactly 0 (periapsis) and the ends exactly +-nuMax.
        if (i * 2 == segments) {
            cn = 1.0;
            sn = 0.0;
        } else if (i % kResync == 0) {
            const double f = static_cast<double>(i) / static_cast<double>(segments);
            const double nu = -nuMax + 2.0 * nuMax * f;
            cn = std::cos(nu);
            sn = std::sin(nu);
        } else {
            const double c2 = cn * cd - sn * sd, s2 = sn * cd + cn * sd;
            cn = c2;
            sn = s2;
        }
        const double r = el.p / (1.0 + el.e * cn);
        put(i, r, cn, sn);
        if (i * 2 != segments) put(segments - i, r, cn, -sn);
    }
    double r2max = 0.0;
    for (const Vec3& p : out.points) {
        const double r2 = p.x * p.x + p.y * p.y;
        if (!std::isfinite(r2)) {
            out.points.clear();
            return false;
        }
        r2max = std::max(r2max, r2);
    }
    out.radiusMax = std::sqrt(r2max);
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
