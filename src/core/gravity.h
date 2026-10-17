#pragma once
#include <vector>
#include "core/bodies.h"

namespace cosmos {

struct Accel {
    std::vector<double> ax, ay, az;
};

// a_i = sum_j mu_j (r_j - r_i) / |r_j - r_i|^3, each pair computed once. Resizes and overwrites a.
void computeAccelerations(const BodyState& s, const std::vector<double>& mu, Accel& a);

} // namespace cosmos
