#pragma once
// Osculating orbits (plan 9.5 and 5.8): the conic of the elements of a state relative to its parent, drawn as a polyline
// of r(nu) = p / (1 + e cos nu) with periapsis and apoapsis marks. GL-free.
//
// The position angle is  phi = omega + sign(h) nu  (plan 5.8: nu = sign(h) (atan2(r_y, r_x) - omega)), so retrograde
// orbits come out mirrored. A bound orbit is the full loop nu in [-pi, pi] (first point = last point). An unbound one
// (e >= 1) is drawn only over the valid range |nu| < arccos(-1/e), cut where r reaches `rCap` (the asymptotes are
// infinitely long); the cut is chosen by the caller from the view so the arms leave the screen.
#include <cstddef>
#include <vector>
#include "core/kepler.h"
#include "core/vec3.h"

namespace cosmos {
namespace app {

constexpr std::size_t kOrbitSegments = 256;

struct OrbitCurve {
    std::vector<Vec3> points;   // relative to the parent, z = 0; segments + 1 points, in the direction of motion
    Vec3 periapsis;             // relative to the parent
    Vec3 apoapsis;              // relative to the parent; only meaningful when hasApoapsis
    bool hasApoapsis = false;   // bound orbits only
    bool closed = false;        // bound: the polyline is a loop
    double radiusMax = 0.0;     // largest distance from the parent of any point (for culling)
};

// False (curve untouched except points cleared) for a degenerate orbit: p <= 0 (a radial path), non-finite elements, or
// e >= 1 with rCap <= q. `rCap` only matters for e >= 1.
bool buildOrbitCurve(const Elements& el, double rCap, std::size_t segments, OrbitCurve& out);

} // namespace app
} // namespace cosmos
