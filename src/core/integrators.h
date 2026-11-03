#pragma once
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"

namespace cosmos {

// One kick-drift-kick leapfrog step. On entry a must hold the accelerations at the current positions
// (call computeAccelerations once to prime it). On return it holds them at the new positions.
void stepKDK(BodyState& s, const std::vector<double>& mu, Accel& a, double dt);

// One Yoshida 4th-order step: three KDK sub-steps with weights w1, w0, w1 (w0 < 0). Same contract for a as stepKDK.
void stepYoshida4(BodyState& s, const std::vector<double>& mu, Accel& a, double dt);

} // namespace cosmos
