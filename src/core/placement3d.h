#pragma once
#include <vector>
#include "core/placement.h"
#include "core/vec3.h"

// 3D Jacobi placement (V1.1 step 5, docs/V1_1_3D_CORE.md). DECLARATIONS ONLY: the developer writes the definitions
// (src/core/placement3d.cpp). The spec is tests/test_3d_core_placement.cpp. Frame and units as kepler3d.h.

namespace cosmos {

// One orbit, as Elements3D minus the derived values. Retrograde is i > pi/2 (there is no flag). Angles in radians:
// i in [0, pi]; Omega, omega any real (taken mod 2 pi); M as for elementsToState3D.
struct OrbitSpec3D {
    double a = 0.0;
    double e = 0.0;
    double i = 0.0;
    double Omega = 0.0;
    double omega = 0.0;
    double M = 0.0;
};

struct PlacementBody3D {
    double mu = 0.0;
    int parent = -1;     // index into the list, -1 for the single root
    OrbitSpec3D orbit;   // ignored for the root
};

// Same algorithm and contract as placeJacobi (placement.h, plan 5.9), with each orbit converted by elementsToState3D
// instead of elementsToState:
//  * each body's orbit is that of its whole subsystem (the body plus everything below it) about the barycentre of its
//    parent plus the parent's inner children's subsystems; siblings are ordered by increasing a (stable for ties);
//  * mu of each conversion is (mu of what is already placed in the parent's system) + (mu of the subsystem);
//  * output index k is body k; positions and velocities in the barycentric frame: sum mu_k r_k = 0 and
//    sum mu_k v_k = 0 (up to rounding), in all three components;
//  * bound orbits only; returns an empty vector for invalid input (not exactly one root, a parent index out of range
//    or equal to the body itself, a body not reachable from the root).
// Reduction: placeJacobi3D(the toOrbitSpec3D of a 2D list) matches placeJacobi(that list) to about 1e-12 relative, with
// z == 0 and vz == 0 EXACTLY for every body (all orbits equatorial).
std::vector<Placed> placeJacobi3D(const std::vector<PlacementBody3D>& bodies);

// A 2D orbit as a 3D one in the ecliptic: i = 0 and omega = s.omega (prograde), or i = pi and omega = -s.omega
// (retrograde); Omega = 0; a, e, M copied. i is exactly 0 or the constant pi (see elementsToState3D).
OrbitSpec3D toOrbitSpec3D(const OrbitSpec& s);

// The whole 2D list converted with toOrbitSpec3D (mu and parent copied).
std::vector<PlacementBody3D> toPlacement3D(const std::vector<PlacementBody>& bodies);

} // namespace cosmos
