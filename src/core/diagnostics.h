#pragma once
#include "core/bodies.h"
#include "core/vec3.h"

namespace cosmos {

// Each quantity is G times the true one, so G never appears (plan section 5.12).
struct Conserved {
    double energyG = 0.0;
    Vec3 momentumG;
    Vec3 angularMomentumG;
};

Conserved conserved(const Bodies& b);

} // namespace cosmos
