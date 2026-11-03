#pragma once
#include "core/bodies.h"
#include "core/vec3.h"

namespace cosmos {

// Atmospheres and exact drag (plan sections 5.5, 5.6). SI units. A body's Atmosphere is {rho0, scaleHeight = H}.

// Density at which an atmosphere ends (plan 5.5, absolute, kg/m^3).
inline constexpr double kAtmosphereCutoffDensity = 1e-6;

// rho(h) = rho0 exp(-h / H) for the height h above the surface. h <= 0 gives rho0 (the shuttle is inside the body, where
// the collision logic has long since taken over). An atmosphere with rho0 <= 0 or H <= 0 has density 0.
double atmosphereDensity(const Atmosphere& a, double h);

// Height at which rho falls to kAtmosphereCutoffDensity: H ln(rho0 / 1e-6). 0 if rho0 <= 1e-6 (no atmosphere to speak of).
double atmosphereTop(const Atmosphere& a);

// Density as used by drag: atmosphereDensity below the top, exactly 0 at and above it (the cutoff of plan 5.5).
double dragDensity(const Atmosphere& a, double h);

// g = mu / R^2 at the surface (m/s^2).
double surfaceGravity(double mu, double radius);

// P0 = rho0 g H, the isothermal-ideal-gas surface pressure (Pa) of plan 5.5.
double surfacePressure(const Atmosphere& a, double mu, double radius);

// Velocity of the air at position r of a body at rb moving with vb and spinning with the signed rate omega about +z:
// v_air = vb + omega z x (r - rb) = vb + omega (-d_y, d_x, 0) (plan 5.5).
inline Vec3 airVelocity(const Vec3& vb, double omega, const Vec3& rb, const Vec3& r)
{
    const Vec3 d = r - rb;
    return {vb.x - omega * d.y, vb.y + omega * d.x, vb.z};
}

// Drag rate k = rho (Cd A) / (2 m) of plan 5.6 (1/m); m is the current mass in kg.
inline double dragRate(double rho, double cdA, double mass) { return rho * cdA / (2.0 * mass); }

// Exact solution of dv/dt = -k |v - v_air| (v - v_air) over dt at fixed rho (plan 5.6):
//   v_rel <- v_rel / (1 + k |v_rel| dt),   v = v_air + v_rel.
// Stable for any dt, k >= 0. The relative speed after dt is v0 / (1 + k v0 dt).
inline Vec3 applyExactDrag(const Vec3& v, const Vec3& vAir, double k, double dt)
{
    const Vec3 rel = v - vAir;
    const double f = 1.0 / (1.0 + k * norm(rel) * dt);
    return vAir + rel * f;
}

} // namespace cosmos
