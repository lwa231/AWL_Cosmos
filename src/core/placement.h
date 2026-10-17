#pragma once
#include <vector>
#include "core/vec3.h"

namespace cosmos {

struct OrbitSpec {
    double a = 0.0;
    double e = 0.0;
    double omega = 0.0;
    double M = 0.0;
    bool retrograde = false;
};

struct PlacementBody {
    double mu = 0.0;
    int parent = -1;   // index into the list, -1 for the single root
    OrbitSpec orbit;   // ignored for the root
};

struct Placed {
    Vec3 r;
    Vec3 v;
};

// Jacobi placement (plan section 5.9). Each body's orbit is that of its whole subsystem about the barycentre
// of its parent plus its inner siblings' subsystems (siblings ordered by increasing a). Bound orbits only.
// The result is in the barycentric frame with zero total momentum. Returns an empty vector for invalid input
// (not exactly one root, bad parent index, unreachable bodies).
std::vector<Placed> placeJacobi(const std::vector<PlacementBody>& bodies);

} // namespace cosmos
