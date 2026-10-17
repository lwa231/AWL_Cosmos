#pragma once
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"

namespace cosmos {

// One kick-drift-kick leapfrog step. On entry a must hold the accelerations at the current positions
// (call computeAccelerations once to prime it). On return it holds them at the new positions.
void stepKDK(BodyState& s, const std::vector<double>& mu, Accel& a, double dt);

} // namespace cosmos
