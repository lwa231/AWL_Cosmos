#pragma once
#include "core/vec3.h"

// Vector spin and full 3D angular momentum in merges (V1.1 step 5, docs/V1_1_3D_CORE.md). DECLARATIONS ONLY: the
// developer writes the definitions (src/core/merge3d.cpp). The spec is tests/test_3d_core_merge.cpp.
//
// Spin representation (chosen here, binding for the 3D core): spin is the body's ANGULAR VELOCITY VECTOR, in rad/s,
// in the ecliptic frame (x-y ecliptic, z up, right-handed). Its direction is the spin axis (right-hand rule) and its
// length the rate. The 2D store's scalar Bodies::spinRate is exactly the z component of this vector, so a 2D body with
// spinRate w is the 3D body with spin (0, 0, w).
// Spin angular momentum (G-scaled, as everywhere in the core: masses are mu = GM):  S_G = k mu R^2 spin,  with k the
// inertia factor (0.4 for a uniform sphere). Orbital angular momentum about a point c:  mu (r - c) x (v - v_c).
//
// These are pure functions on plain values, so the developer can write and test the physics without touching the body
// store. Wiring them into mergeBodies (a Vec3 spin in Bodies) is the integration step after the spec passes.

namespace cosmos {

// One body as the merge sees it. SI units: mu m^3/s^2, radius m, r m, v m/s, spin rad/s.
struct SpinBody3D {
    double mu = 0.0;
    double radius = 0.0;
    double inertiaFactor = 0.4;  // k in I = k m R^2
    Vec3 r;
    Vec3 v;
    Vec3 spin;                   // angular velocity vector, rad/s
};

// What a perfectly inelastic 3D merge produces.
struct Merge3D {
    double mu = 0.0;          // mu_1 + mu_2
    double radius = 0.0;      // the mergedRadius argument, copied
    double inertiaFactor = 0.0; // the survivor's
    Vec3 r;                   // centre of mass (plain mean if both are massless)
    Vec3 v;                   // centre-of-mass velocity (plain mean if both are massless)
    Vec3 spin;                // angular velocity of the merged body, rad/s, after the breakup clamp

    Vec3 spinAngularMomentumG; // S_G = S_1 + S_2 + sum mu_k (r_k - r) x (v_k - v): everything the merged body must carry
    Vec3 omegaRaw;            // S_G / I_G before the clamp, I_G = k_survivor (mu_1 + mu_2) R^2
    double omegaBreak = 0.0;  // breakupSpin(mu, R) = sqrt(mu / R^3) (radii.h); 0 when R = 0
    bool spinClamped = false; // |omegaRaw| > omegaBreak: spin = omegaRaw scaled to length omegaBreak (same direction)
    // Angular momentum the merged body does not carry: S_G - I_G spin. EXACTLY (0, 0, 0) when not clamped and I_G > 0
    // (in 3D the off-plane orbital part goes into the spin vector instead of being lost, unlike the 2D merge).
    // When I_G = 0 (R = 0 or massless) spin = 0 and lost = S_G. Total L about any point is conserved:
    //   L_before == L_after + lostAngularMomentumG, up to rounding.
    Vec3 lostAngularMomentumG;
};

// Spin angular momentum k mu R^2 spin of one body (G-scaled).
Vec3 spinAngularMomentumG(const SpinBody3D& b);

// Total angular momentum of one body about the origin: mu (r x v) + spinAngularMomentumG(b).
Vec3 angularMomentumG(const SpinBody3D& b);

// Merge `absorbed` into `survivor` perfectly inelastically (plan 5.4 in 3D). The caller chooses the survivor (the
// heavier body, lower ID on a tie, as mergeBodies does) and the merged radius (volumes add, cbrt(R1^3 + R2^3), or the
// Schwarzschild radius for a black hole: kind rules stay in mergeBodies). Conserves mass (mu), linear momentum,
// the centre of mass and total 3D angular momentum (orbital + spin, with the clamp loss reported). Uses the same
// operation order as mergeBodies for the shared quantities (x, v, the orbital cross products), so:
// Reduction to 2D: for two bodies with r.z = v.z = 0 and spin = (0, 0, w), the result matches mergeBodies on the same
// pair to about 1e-12 relative: mu, r, v equal, spin == (0, 0, spinRate), omegaRaw.z == MergeResult::omegaRaw,
// omegaBreak and spinClamped equal, lost.z == MergeResult::lostAngularMomentumG.z, lost.x == lost.y == 0.
// For an off-plane collision without a clamp, I_G * spin.x and I_G * spin.y equal the x/y angular momentum the 2D
// merge reports as lost (that is what the vector spin now keeps).
Merge3D mergeSpin3D(const SpinBody3D& survivor, const SpinBody3D& absorbed, double mergedRadius);

} // namespace cosmos
