#pragma once
#include "core/vec3.h"

namespace cosmos {

// Dormand-Prince 5(4) with 7 stages and the first-same-as-last property (plan section 6.4) for one massless particle:
//     r' = v,   v' = a(t, r).
// The acceleration depends on time and position only (drag, thrust and torque are split operators around the step,
// see shuttles.h), which is what makes the first stage of a step equal to the last stage of the previous one.
// Coefficients: Dormand & Prince (1980), J. Comp. Appl. Math. 6, 19; Hairer, Norsett & Wanner, Solving ODEs I, II.5.

namespace dopri5 {

// Nodes c_i
inline constexpr double c[7] = {0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0};

// Runge-Kutta matrix a_ij (row i = stage, columns j < i)
inline constexpr double a[7][6] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0},
    {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0},
    {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0, 0.0},
    {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0, 0.0},
    {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0}};

// Fifth-order weights: they equal the last row of a (that is the FSAL property), b5[6] = 0.
inline constexpr double b5[7] = {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0, 0.0};

// Embedded fourth-order weights
inline constexpr double b4[7] = {5179.0 / 57600.0, 0.0, 7571.0 / 16695.0, 393.0 / 640.0, -92097.0 / 339200.0,
                                 187.0 / 2100.0,   1.0 / 40.0};

// Error weights e_i = b5_i - b4_i, written out as the reduced fractions of Hairer et al. (checked against b5 - b4 in
// the tests).
inline constexpr double e[7] = {71.0 / 57600.0, 0.0, -71.0 / 16695.0, 71.0 / 1920.0, -17253.0 / 339200.0, 22.0 / 525.0,
                                -1.0 / 40.0};

} // namespace dopri5

struct Dopri5State {
    Vec3 r;
    Vec3 v;
};

struct Dopri5Result {
    Dopri5State y1;   // fifth-order solution at t + h (local extrapolation)
    Dopri5State err;  // y1 minus the embedded fourth-order solution (position part, velocity part)
    Vec3 a1;          // acceleration at (t + h, y1.r): the first stage of the next step (FSAL)
};

// One Dormand-Prince step of length h from y0 at time t. a0 is the acceleration at (t, y0.r) (the FSAL value, so the
// caller evaluates it once and then reuses result.a1). accel(t, r) -> Vec3 is any callable; it is called six times.
// Fixed summation order, no allocation, no global state.
template <class AccelFn>
Dopri5Result dopri5Step(const AccelFn& accel, double t, double h, const Dopri5State& y0, const Vec3& a0)
{
    Vec3 kr[7], kv[7]; // stage derivatives: kr = stage velocity, kv = stage acceleration
    kr[0] = y0.v;
    kv[0] = a0;
    Vec3 rEnd; // position of the last stage = the fifth-order solution
    for (int i = 1; i < 7; ++i) {
        Vec3 sr, sv; // sum_j a_ij k_j
        for (int j = 0; j < i; ++j) {
            const double w = dopri5::a[i][j];
            if (w == 0.0) continue;
            sr += kr[j] * w;
            sv += kv[j] * w;
        }
        const Vec3 ri = y0.r + sr * h;
        kr[i] = y0.v + sv * h; // stage velocity
        kv[i] = accel(t + dopri5::c[i] * h, ri);
        if (i == 6) rEnd = ri;
    }
    // Stage 7 (index 6) is evaluated at the fifth-order solution: ri = y1.r, kr[6] = y1.v, kv[6] = a(t + h, y1.r)
    Dopri5Result out;
    out.y1.r = rEnd;
    out.y1.v = kr[6];
    out.a1 = kv[6];
    Vec3 er, ev;
    for (int j = 0; j < 7; ++j) {
        if (dopri5::e[j] == 0.0) continue;
        er += kr[j] * dopri5::e[j];
        ev += kv[j] * dopri5::e[j];
    }
    out.err.r = er * h;
    out.err.v = ev * h;
    return out;
}

// Tolerances of the error norm (plan 6.4). The error of a step is the RMS over the six components of
//     err_i / (atol_i + rtol * |y_i|),
// where, for a shuttle orbiting a body, |y_i| is not the barycentric coordinate (which can be a thousand times the
// orbit radius) but the scale of the orbit: the distance d to the dominant body for the position components and the
// local circular speed sqrt(mu / d) for the velocity components. So a low orbit is integrated as accurately, relative
// to its own size, as a wide one.
struct Dopri5Tolerance {
    // Relative tolerance per step. Measured (tests/test_dopri5_orbits.cpp, Domus-like mu, 100 orbits in one call): at 1e-10
    // a circular orbit ends within 2.6e-5 and an e = 0.9 orbit within 5e-5 of the orbit's size (mostly phase drift),
    // with ~140 and ~280 steps per revolution and a relative energy drift of 5e-8 and 1e-7. Each decade of rtol buys
    // about 6.5x in accuracy for 1.6x the steps. The numbers are the same for a planet 1.45e10 m from the origin,
    // which is the point of the distance scaling below.
    double rtol = 1e-10;
    // Absolute floors, so the scale never reaches zero: 1 um and 1 nm/s.
    double atolPos = 1e-6;
    double atolVel = 1e-9;
};

// Scales of the error norm for a particle at distance d from a body with gravitational parameter mu.
inline double dopri5PositionScale(const Dopri5Tolerance& tol, double d) { return tol.atolPos + tol.rtol * d; }
double dopri5VelocityScale(const Dopri5Tolerance& tol, double mu, double d);

// RMS error norm sqrt(sum (err_i / scale_i)^2 / 6) with one position scale and one velocity scale. The step is
// acceptable when this is <= 1. Returns +infinity for a non-finite error (so the step is rejected).
double dopri5ErrorNorm(const Dopri5State& err, double posScale, double velScale);

// Step-size factor min(5, max(0.2, 0.9 err^(-1/5))) of plan 6.4; err = 0 gives 5, a non-finite err gives 0.2.
double dopri5StepFactor(double err);

} // namespace cosmos
