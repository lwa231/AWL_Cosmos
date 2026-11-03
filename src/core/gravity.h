#pragma once
#include <vector>
#include "core/bodies.h"

namespace cosmos {

struct Accel {
    std::vector<double> ax, ay, az;
};

// a_i = sum_j mu_j (r_j - r_i) / |r_j - r_i|^3, each pair computed once. Resizes and overwrites a.
void computeAccelerations(const BodyState& s, const std::vector<double>& mu, Accel& a);

// Same, with the overlap field of plan section 5.3. For a pair closer than R_c = max(R_i, R_j) the uniform-sphere
// interior field a = mu d / R_c^3 replaces 1/r^2 (one symmetric R_c per pair, so Newton's third law holds).
// With every radius 0 this performs exactly the same operations as the point-mass overload (bit-identical).
void computeAccelerations(const BodyState& s, const std::vector<double>& mu, const std::vector<double>& radius, Accel& a);

} // namespace cosmos
