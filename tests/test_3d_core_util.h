#pragma once
// Helpers shared by the [3d-core] spec tests (tests/test_3d_core_*.cpp). Header-only, test code only.
#include <algorithm>
#include <cmath>
#include <random>
#include "core/vec3.h"

namespace core3d_test {

using cosmos::Vec3;

const double kPi = 3.141592653589793238462643383279502884;
const double kDeg = kPi / 180.0;

// Smallest signed difference a - b of two angles, in (-pi, pi]
inline double angleDiff(double a, double b)
{
    double t = std::fmod(a - b + kPi, 2.0 * kPi);
    if (t <= 0.0) t += 2.0 * kPi;
    return t - kPi;
}

inline bool near(double x, double ref, double tol)
{
    return std::abs(x - ref) <= tol * std::max(1.0, std::abs(ref));
}

inline double relErr(const Vec3& a, const Vec3& ref)
{
    const double n = cosmos::norm(ref);
    return n > 0.0 ? cosmos::norm(a - ref) / n : cosmos::norm(a);
}

// Rotations by angle t (right-handed, active: they turn the vector, not the frame)
inline Vec3 rotX(const Vec3& a, double t)
{
    const double c = std::cos(t), s = std::sin(t);
    return {a.x, c * a.y - s * a.z, s * a.y + c * a.z};
}
inline Vec3 rotZ(const Vec3& a, double t)
{
    const double c = std::cos(t), s = std::sin(t);
    return {c * a.x - s * a.y, s * a.x + c * a.y, a.z};
}

// A fixed, generic rotation Q (no axis along x, y or z), and its use on a vector
inline Vec3 rotQ(const Vec3& a) { return rotZ(rotX(rotZ(a, 0.7), 1.1), -2.3); }

// Fixed-seed, platform-independent uniform in [0, 1)
inline double uni(std::mt19937_64& g)
{
    return static_cast<double>(g() >> 11) * (1.0 / 9007199254740992.0);
}

} // namespace core3d_test
