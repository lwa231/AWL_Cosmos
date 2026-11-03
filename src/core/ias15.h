#pragma once
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"

namespace cosmos {

namespace ias15 {

// Gauss-Radau nodes in [0, 1): h[0] = 0, h[1..7] are the roots used inside one step (tau = t / dt).
extern const double h[8];

// Divided differences: F[0..7] are the accelerations at the nodes (F[0] = a0), g[1..7] are filled (g[0] unused).
void gFromF(const double F[8], double g[8]);

// Convert g[1..7] to the monomial coefficients b[0..6] of a(tau) = a0 + b0*tau + b1*tau^2 + ... + b6*tau^7.
void bFromG(const double g[8], double b[7]);

// Position / velocity of one component at tau = hh (0..1) inside a step of length dt, from the step-start values
// x0, v0, a0 and the polynomial coefficients b[0..6]. hh = 1 gives the end of the step.
double positionAt(double x0, double v0, double a0, const double b[7], double hh, double dt);
double velocityAt(double v0, double a0, const double b[7], double hh, double dt);

} // namespace ias15

// One fixed-step IAS15 step. Same contract for a as stepKDK: valid on entry, valid at the new positions on return.
// Returns the number of predictor-corrector rounds used.
int stepIAS15Fixed(BodyState& s, const std::vector<double>& mu, Accel& a, double dt);

} // namespace cosmos
