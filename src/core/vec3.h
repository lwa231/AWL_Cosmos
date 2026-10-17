#pragma once
#include <cmath>

namespace cosmos {

// Three doubles. Used for formulas that read better as vectors; the body store itself keeps x/y/z in separate arrays.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

inline constexpr Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline constexpr Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
inline constexpr Vec3 operator/(const Vec3& a, double s) { return {a.x / s, a.y / s, a.z / s}; }

inline Vec3& operator+=(Vec3& a, const Vec3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline Vec3& operator-=(Vec3& a, const Vec3& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }

// Summation order is fixed as (x*x + y*y) + z*z everywhere, so results are reproducible (determinism rules, plan section 4.6).
inline constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Right-handed. The z component equals a.x*b.y - a.y*b.x, the same form as h = r_x v_y - r_y v_x in plan section 5.8.
inline constexpr Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

// Squared length: no square root, used where only comparisons or |d|^2 are needed.
inline constexpr double norm2(const Vec3& a) { return dot(a, a); }

// Length. sqrt of dot(a, a): one correctly rounded square root (no std::hypot).
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

} // namespace cosmos
