#include "core/kepler.h"
#include <cmath>
#include <limits>

namespace cosmos {

namespace {
const double kPi = 3.141592653589793238462643383279502884;

// Wrap to (-pi, pi]
double wrapPi(double a)
{
    double t = std::fmod(a + kPi, 2.0 * kPi);
    if (t <= 0.0) t += 2.0 * kPi;
    return t - kPi;
}

double solveEllipticKepler(double M, double e)
{
    M = wrapPi(M);
    double E = (e > 0.8) ? std::copysign(kPi, M) : M;
    for (int i = 0; i < 50; ++i) {
        const double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < 1e-14) break;
    }
    return E;
}

double solveHyperbolicKepler(double M, double e)
{
    double H = std::asinh(M / e);
    for (int i = 0; i < 50; ++i) {
        const double dH = (e * std::sinh(H) - H - M) / (e * std::cosh(H) - 1.0);
        H -= dH;
        if (std::abs(dH) < 1e-14) break;
    }
    return H;
}
}

Elements stateToElements(const Vec3& r, const Vec3& v, double mu)
{
    Elements el;

    const double rm = std::sqrt(r.x * r.x + r.y * r.y);
    const double v2 = v.x * v.x + v.y * v.y;
    const double rv = r.x * v.x + r.y * v.y;

    el.h = r.x * v.y - r.y * v.x;
    el.retrograde = el.h < 0.0;
    const double sgn = el.retrograde ? -1.0 : 1.0;

    const double energy = 0.5 * v2 - mu / rm;
    el.a = -mu / (2.0 * energy);

    const double ex = ((v2 - mu / rm) * r.x - rv * v.x) / mu;
    const double ey = ((v2 - mu / rm) * r.y - rv * v.y) / mu;
    el.e = std::sqrt(ex * ex + ey * ey);

    const double theta = std::atan2(r.y, r.x);
    if (el.e < 1e-10) {
        el.e = 0.0;
        el.omega = theta;
    } else {
        el.omega = std::atan2(ey, ex);
    }

    el.p = el.h * el.h / mu;
    el.q = el.p / (1.0 + el.e);
    el.nu = wrapPi(sgn * (theta - el.omega));

    if (el.e < 1.0) {
        const double E = 2.0 * std::atan2(std::sqrt(1.0 - el.e) * std::sin(0.5 * el.nu),
                                          std::sqrt(1.0 + el.e) * std::cos(0.5 * el.nu));
        el.M = E - el.e * std::sin(E);
    } else if (el.e > 1.0) {
        const double H = 2.0 * std::atanh(std::sqrt((el.e - 1.0) / (el.e + 1.0)) * std::tan(0.5 * el.nu));
        el.M = el.e * std::sinh(H) - H;
    } else {
        el.M = std::numeric_limits<double>::quiet_NaN();
    }
    return el;
}

RelState elementsToState(const Elements& el, double mu)
{
    double x, y, vx, vy;

    if (el.e < 1.0) {
        const double E = solveEllipticKepler(el.M, el.e);
        const double cE = std::cos(E), sE = std::sin(E);
        const double b = std::sqrt(1.0 - el.e * el.e);
        const double r = el.a * (1.0 - el.e * cE);
        const double k = std::sqrt(mu * el.a) / r;
        x = el.a * (cE - el.e);
        y = el.a * b * sE;
        vx = -k * sE;
        vy = k * b * cE;
    } else {
        const double H = solveHyperbolicKepler(el.M, el.e);
        const double cH = std::cosh(H), sH = std::sinh(H);
        const double b = std::sqrt(el.e * el.e - 1.0);
        const double r = el.a * (1.0 - el.e * cH);
        const double k = std::sqrt(-mu * el.a) / r;
        x = el.a * (cH - el.e);
        y = -el.a * b * sH;
        vx = -k * sH;
        vy = k * b * cH;
    }

    if (el.retrograde) {
        y = -y;
        vy = -vy;
    }

    const double c = std::cos(el.omega), s = std::sin(el.omega);
    RelState out;
    out.r = {c * x - s * y, s * x + c * y, 0.0};
    out.v = {c * vx - s * vy, s * vx + c * vy, 0.0};
    return out;
}

} // namespace cosmos
