#pragma once
#include "core/vec3.h"

namespace cosmos {

struct Elements {
    double a = 0.0;
    double e = 0.0;
    double p = 0.0;
    double q = 0.0;
    double h = 0.0;
    double omega = 0.0;
    double nu = 0.0;
    double M = 0.0;
    bool retrograde = false;
};

struct RelState {
    Vec3 r;
    Vec3 v;
};

// r and v are the body's state relative to its parent; mu = mu_parent + mu_body. 2D: only x and y are used.
// e is snapped to 0 below 1e-10 (omega then points at the body). M is NaN for an exactly parabolic orbit.
Elements stateToElements(const Vec3& r, const Vec3& v, double mu);

// Uses el.a, el.e, el.omega, el.M and el.retrograde (e < 1 with a > 0, or e > 1 with a < 0). mu as above.
RelState elementsToState(const Elements& el, double mu);

} // namespace cosmos
