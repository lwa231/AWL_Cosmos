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

// Pair potential energy times G: -mu_i mu_j / r, or with the plan section 5.3 interior potential
// -mu_i mu_j (3R^2 - r^2) / (2R^3) when r < rc (rc = max(R_i, R_j)). rc = 0 gives the point-mass value exactly.
double pairPotentialG(double mui, double muj, double r, double rc);

// Kinetic + all pair potentials (overlap-aware). Spin energy is not part of the plan's E.G.
double energyG(const Bodies& b);

Conserved conserved(const Bodies& b);

// Running totals of what merges removed, so the diagnostics chart can show integrator error only (plan 5.4, 5.12).
// energyG sums dE.G = E.G(after) - E.G(before); lostAngularMomentumG sums the L.G a merge could not keep.
struct MergeLedger {
    double energyG = 0.0;
    Vec3 lostAngularMomentumG;
};

// Add the logged merge changes back: E - sum(dE), L + sum(lost L). P is untouched (merges conserve it).
Conserved withMergesAddedBack(const Conserved& c, const MergeLedger& ledger);

} // namespace cosmos
