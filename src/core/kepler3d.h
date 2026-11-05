#pragma once
#include "core/kepler.h"
#include "core/vec3.h"

// 3D Kepler elements (V1.1 step 5, docs/V1_1_3D_CORE.md). DECLARATIONS ONLY: the developer writes the definitions
// (src/core/kepler3d.cpp). The spec is tests/test_3d_core_kepler.cpp.
//
// Frame: the ecliptic is the x-y plane, z points up (+z is the north pole of the ecliptic), right-handed. Every 2D
// scene lives at z = 0 and must stay there exactly. SI units: metres, m/s, mu = GM in m^3/s^2, angles in radians.
//
// The usual 3-1-3 rotation takes the perifocal frame (x_p to periapsis, y_p 90 degrees ahead in the direction of
// motion, z_p along h) to the ecliptic frame:  r = R3(-Omega) R1(-i) R3(-omega) r_perifocal  (Vallado 2.6,
// Curtis 4.6). The orbit normal is then  h / |h| = (sin i sin Omega, -sin i cos Omega, cos i).

namespace cosmos {

// Elements of a two-body conic in 3D. Same meaning as the 2D Elements (kepler.h) for a, e, p, q, nu and M; the plane
// is given by i and Omega instead of the retrograde flag.
struct Elements3D {
    double a = 0.0;     // semi-major axis, m. a > 0 bound (e < 1), a < 0 hyperbolic (e > 1), +-inf parabolic
    double e = 0.0;     // eccentricity >= 0. Snapped to exactly 0 below 1e-10, like stateToElements
    double p = 0.0;     // semi-latus rectum h^2 / mu, m
    double q = 0.0;     // periapsis p / (1 + e), m
    double h = 0.0;     // |r x v|, m^2/s. Never negative (the 2D Elements::h is signed; here the sign lives in i)
    double i = 0.0;     // inclination, [0, pi]. i > pi/2 is retrograde (the 2D retrograde flag at z = 0 is i = pi)
    double Omega = 0.0; // longitude of the ascending node, [0, 2 pi), measured from +x towards +y in the ecliptic
    double omega = 0.0; // argument of periapsis, [0, 2 pi), from the ascending node to periapsis in the direction of motion
    double nu = 0.0;    // true anomaly, (-pi, pi], from periapsis in the direction of motion (as in 2D)
    double M = 0.0;     // mean anomaly: (-pi, pi] for an ellipse, any real for a hyperbola, NaN if exactly parabolic
};

// State relative to the parent -> elements. r, v relative to the parent; mu = mu_parent + mu_body.
//
// Degenerate cases (the conversion must never produce NaN for a non-radial orbit):
//  * Equatorial: when sqrt(h_x^2 + h_y^2) <= 1e-10 |h| (sin i tiny) the node line is undefined. Then i is set to
//    EXACTLY 0 (h_z > 0) or EXACTLY pi (h_z < 0), Omega = 0, and omega is measured from +x (the node is taken to be
//    the x axis). With i = pi the R1(-pi) flip reverses the y axis, so omega = -(angle of periapsis from +x), wrapped
//    to [0, 2 pi): that is the angle from +x to periapsis measured in the direction of motion.
//  * Circular: e < 1e-10 gives e = 0 and omega = 0; nu is then the argument of latitude u (from the ascending node to
//    r in the direction of motion). Circular and equatorial: Omega = omega = 0 and nu is the true longitude, from +x
//    in the direction of motion (so -theta for i = pi).
//  * Radial (h = 0): the plane is undefined; out of scope (no spec test). Any finite result is acceptable.
//  * Exactly parabolic (e = 1): M = NaN, as in 2D.
//
// Reduction to 2D (r.z = v.z = 0), against el2 = stateToElements(r, v, mu), to about 1e-12 relative:
//  a, e, p, q, nu, M equal; h == |el2.h|; i is exactly 0 (prograde) or exactly pi (el2.retrograde); Omega == 0;
//  omega == el2.omega (prograde) or -el2.omega (retrograde), mod 2 pi. When e = 0 the 2D code points omega at the
//  body (nu = 0) while 3D sets omega = 0, so there nu == el2.omega + el2.nu (prograde) or el2.nu - el2.omega
//  (retrograde), mod 2 pi, instead.
Elements3D stateToElements3D(const Vec3& r, const Vec3& v, double mu);

// Elements -> state relative to the parent. Uses el.a, el.e, el.i, el.Omega, el.omega and el.M (not nu, p, q, h).
// Valid input: e < 1 with a > 0, or e > 1 with a < 0 (as elementsToState). mu as above.
// Must be exactly planar when the orbit is equatorial: for el.i == 0 or el.i == pi (the constant 3.14159..., as
// stateToElements3D writes it) the result has r.z == 0 and v.z == 0 EXACTLY (std::sin(pi) is 1.2e-16, not 0, so the
// equatorial case needs sin i = 0 explicitly). This keeps 2D scenes in the plane bit for bit.
// Reduction: elementsToState3D(toElements3D(el2), mu) matches elementsToState(el2, mu) to about 1e-12 relative.
RelState elementsToState3D(const Elements3D& el, double mu);

// The 2D elements as 3D ones in the ecliptic: i = 0, Omega = 0, omega = el2.omega wrapped to [0, 2 pi) for a prograde
// orbit; i = pi, Omega = 0, omega = -el2.omega wrapped to [0, 2 pi) for a retrograde one. a, e, p, q, nu, M copied,
// h = |el2.h|. Used by placement and by every "reduces to 2D" test.
Elements3D toElements3D(const Elements& el2);

// Unit orbit normal h / |h| from i and Omega: (sin i sin Omega, -sin i cos Omega, cos i). Exactly (0, 0, 1) for i == 0
// and (0, 0, -1) for i == pi (same exact-zero rule as elementsToState3D).
Vec3 orbitNormal(const Elements3D& el);

} // namespace cosmos
