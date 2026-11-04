#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "core/bodies.h"
#include "core/controllers.h"
#include "core/dopri5.h"
#include "core/events.h"
#include "core/ias15.h"
#include "core/scene.h"
#include "core/vec3.h"

namespace cosmos {

// Shuttle physics (plan sections 5.5-5.7, 6.4, 7.1-7.3). A shuttle is massless to the bodies: it only feels their
// gravity, so each shuttle is advanced on its own against the bodies' dense output, across one body step at a time.
// Masses here are kg (dry + fuel); body masses stay mu = GM. No OpenGL; deterministic (fixed operation order, no
// global state): the result for one shuttle does not depend on the other shuttles or on the thread that runs it.

using ShuttleId = std::uint64_t;

// ---------------------------------------------------------------------------------------------------------------
// Pure operators (exposed so they can be tested in isolation; propagateShuttle composes them)

// Reaction wheels (plan 5.7): I domega/dt = tau_max * turn at constant torque, so over dt omega is linear and theta
// quadratic. alpha = tau_max * turn / I. This is exact, not an approximation.
struct Rotation {
    double theta = 0.0;
    double omega = 0.0;
};
inline Rotation advanceRotation(double theta, double omega, double alpha, double dt)
{
    return {theta + omega * dt + 0.5 * alpha * dt * dt, omega + alpha * dt};
}

// Result of a burn of the given duration at constant throttle (plan 5.7).
struct BurnResult {
    double dv = 0.0;        // magnitude of the velocity change along the heading, m/s
    double burnTime = 0.0;  // how long the engine actually ran (shorter than duration if the fuel ran out), s
    double fuelUsed = 0.0;  // kg
    bool emptied = false;   // the fuel ran out during this burn (never with infinite fuel)
};

// Exact rocket equation over a substep with constant throttle: with mdot = throttle * maxThrust / ve,
//   dv = ve ln(m0 / m1), m0 = dry + fuel, m1 = m0 - mdot * burnTime   (computed with log1p, so tiny burns stay exact).
// If the fuel runs out first, the burn stops there (m1 = dry). With infiniteFuel the mass does not change and
// dv = throttle * maxThrust * duration / m0. Returns zeros for throttle <= 0, no fuel (finite case), or ve <= 0.
// throttle is clamped to [0, 1]. dryMass must be > 0.
BurnResult rocketBurn(double dryMass, double fuelMass, double throttle, double maxThrust, double exhaustVelocity,
                      double duration, bool infiniteFuel);

// ---------------------------------------------------------------------------------------------------------------
// Store

// Plain data for one shuttle. Defaults are the table of plan 7.1.
struct ShuttleDesc {
    ShuttleId id = 0;
    std::string name;
    ControllerKind controller = ControllerKind::Coast;
    Vec3 position;
    Vec3 velocity;
    double heading = 0.0;        // theta, rad
    double spin = 0.0;           // omega_s, rad/s
    double dryMass = 2000.0;     // kg, must be > 0
    double fuelMass = 3000.0;    // kg, >= 0
    double maxThrust = 60000.0;  // N
    double exhaustVelocity = 3000.0; // m/s
    double cdA = 2.0;            // m^2
    double radius = 2.0;         // collision radius, m
    double maxTorque = 5000.0;   // N m
    double inertia = 1.0e4;      // kg m^2
};

// A resolved scene shuttle as a ShuttleDesc: the scene's parameters, the resolved state, and the heading along the
// absolute velocity (the scene has no heading field; Simulation replaces it by the direction of the velocity relative to
// the shuttle's parent, so a shuttle orbiting a moving planet starts prograde). Returns false if the controller name is
// unknown.
bool shuttleDescFromResolved(const ResolvedShuttle& r, ShuttleDesc& out);

inline constexpr std::int64_t kTickUnset = std::numeric_limits<std::int64_t>::min();

// Structure-of-arrays store (plan 4.2). IDs are permanent uint64 values supplied by the caller (scene IDs, plan 4.3):
// an ID that was ever added is never accepted again, even after the shuttle is removed. Dead shuttles stay in the store
// (alive = 0) until compact(), so indices are stable during a run.
class Shuttles {
public:
    // false (nothing added) for id 0, an ID already used, dryMass <= 0, or non-finite/negative parameters.
    bool add(const ShuttleDesc& d);
    bool addResolved(const ResolvedShuttle& r); // shuttleDescFromResolved + add
    std::optional<std::size_t> find(ShuttleId id) const;
    std::size_t size() const { return ids.size(); }
    std::size_t aliveCount() const;
    void kill(std::size_t i) { alive[i] = 0; }
    // Removes dead shuttles, keeping the order of the rest. Their IDs stay retired.
    void compact();
    bool consistent() const;

    double mass(std::size_t i) const { return dryMass[i] + fuelMass[i]; }

    // Global switch (plan 5.7): no fuel is used, thrust never runs out.
    bool infiniteFuel = false;

    BodyState state; // x, y, z, vx, vy, vz (same layout as the bodies)
    std::vector<ShuttleId> ids;
    std::vector<std::string> name;
    std::vector<ControllerKind> controller;
    std::vector<double> heading, spin;
    std::vector<double> dryMass, fuelMass;
    std::vector<double> maxThrust, exhaustVelocity, cdA, radius, maxTorque, inertia;
    std::vector<double> throttle, turn;    // the held command
    std::vector<std::uint8_t> alive;
    std::vector<std::uint8_t> fuelEmptyReported;
    // Index of the next controller tick to evaluate (kTickUnset until the first propagation, which then evaluates the
    // controller at its start time) and the integrator's proposal for the next step size (0 = choose automatically).
    std::vector<std::int64_t> nextTick;
    std::vector<double> nextStep;

private:
    std::unordered_set<ShuttleId> used_;
    std::unordered_map<ShuttleId, std::size_t> index_;
    void reindex();
};

// ---------------------------------------------------------------------------------------------------------------
// Bodies as seen by a shuttle

// Where the bodies are at any time inside the current body step. Calls are const, allocation-free and touch no shared
// state, so many threads may query one object at the same time. Body index = index in the Bodies store.
class IBodyEphemeris {
public:
    virtual ~IBodyEphemeris() = default;
    virtual std::size_t count() const = 0;
    virtual Vec3 position(std::size_t body, double t) const = 0;
    virtual Vec3 velocity(std::size_t body, double t) const = 0;
};

// The last accepted step of an IAS15 integrator (its dense output, plan 6.3/6.6). Valid for times in
// [stepStartTime(), stepEndTime()] and until the integrator steps again; the object only holds a reference.
class Ias15Ephemeris final : public IBodyEphemeris {
public:
    explicit Ias15Ephemeris(const IAS15& ias) : ias_(ias) {}
    std::size_t count() const override { return ias_.bodyCount(); }
    Vec3 position(std::size_t body, double t) const override { return ias_.positionOf(body, ias_.tauAt(t)); }
    Vec3 velocity(std::size_t body, double t) const override { return ias_.velocityOf(body, ias_.tauAt(t)); }
    double startTime() const { return ias_.stepStartTime(); }
    double endTime() const { return ias_.stepEndTime(); }

private:
    const IAS15& ias_;
};

// Bodies moving in straight lines from their state in a Bodies store: r(t) = r + v (t - tRef). With zero velocities
// they are static. The store is read live (it is not copied), so do not modify it while shuttles are propagated.
class LinearEphemeris final : public IBodyEphemeris {
public:
    explicit LinearEphemeris(const Bodies& b, double tRef = 0.0) : b_(b), tRef_(tRef) {}
    std::size_t count() const override { return b_.size(); }
    Vec3 position(std::size_t i, double t) const override
    {
        const double dt = t - tRef_;
        return {b_.state.x[i] + b_.state.vx[i] * dt, b_.state.y[i] + b_.state.vy[i] * dt, b_.state.z[i] + b_.state.vz[i] * dt};
    }
    Vec3 velocity(std::size_t i, double) const override { return {b_.state.vx[i], b_.state.vy[i], b_.state.vz[i]}; }

private:
    const Bodies& b_;
    double tRef_;
};

// The node a shuttle orbits, for the controllers' observation (plan 5.9 / 15 item 2c: the hierarchy node with the smallest
// Hill sphere that contains the shuttle, a body before a barycentre node of the same radius, else the system barycentre).
// id 0 means a barycentre node.
struct ParentRef {
    BodyId id = 0;
    double mu = 0.0;
    Vec3 r;
    Vec3 v;
};

class IParentProvider {
public:
    virtual ~IParentProvider() = default;
    virtual ParentRef parentOf(double t, const Vec3& shuttlePosition) const = 0;
};

// Everything a propagation needs besides the shuttle store: read-only, shared by all threads.
// Build one per body-step interval (or per frame); the references must outlive it.
class ShuttleEnvironment {
public:
    // ephemeris.count() must equal bodies.size().
    ShuttleEnvironment(const Bodies& b, const IBodyEphemeris& e, const ControllerSet& c);

    const Bodies& bodies;
    const IBodyEphemeris& ephemeris;
    const ControllerSet& controllers;

    // Parent of a shuttle for the observation. nullptr = the body with the strongest pull on the shuttle (largest
    // mu / d^2), which is also what the integrator's error scale uses.
    const IParentProvider* parents = nullptr;

    // Controller tick period (plan 4.6, Scene settings.controllerTick_s): controllers run at times k * controllerTick_s.
    double controllerTick = 0.1;

    // Error control of the gravity step (see Dopri5Tolerance).
    Dopri5Tolerance tolerance;

    // While the thrust is on, or inside an atmosphere, the step is limited to this fraction of the local dynamical time
    // sqrt(d^3 / mu) of the dominant body. The operators are applied around the gravity step (second-order splitting) and
    // the DOPRI error estimate does not see their error, so this limit controls it. Measured for a 100 s full-thrust burn
    // at a fixed inertial heading from a circular orbit (dynamical time 369 s): the error of the final state is
    // 20 m / 4.4e-2 m/s at 0.02, 1.3 m / 2.8e-3 m/s at 0.005 and 0.2 m / 4.5e-4 m/s at 0.002, falling as the fraction
    // squared. A controller that changes its command every tick (any state-dependent autopilot) never takes steps longer
    // than 0.1 s anyway. 0 disables the limit.
    double activeStepFraction = 0.005;
    // Inside an atmosphere the step is also limited to this many scale heights of travel at the speed relative to the
    // body (|v_rel| >= 1 m/s), so the split drag samples the exponential density profile finely enough: the drag
    // operator uses the density at the two ends of the gravity step, a trapezoid rule with relative error
    // (travel / H)^2 / 12. 0.1 gives 1e-3 of the drag impulse of a step (measured in the atmosphere tests). 0 disables.
    double atmosphereResolution = 0.1;
    // While thrusting the step is also limited so the heading turns by at most this many rad (the thrust direction is
    // taken at the middle of each half step).
    double maxHeadingChangePerStep = 0.1;

    // Smallest step the controller may shrink to, s. A step this short is accepted whatever its error estimate.
    double minStep = 1e-9;

    // Per body: height of the atmosphere top (0 without an atmosphere). Computed at construction.
    std::vector<double> atmosphereTop;
};

// ---------------------------------------------------------------------------------------------------------------
// Propagation

struct ShuttleStats {
    long acceptedSteps = 0;
    long rejectedSteps = 0;
    long controllerTicks = 0;
    long forceCalls = 0;
};

// One accepted step of one shuttle. r0/v0 are the state at t0 and r1/v1 at t1 (the shuttle's whole motion in the step,
// including the thrust and drag operators, which change v but never r). A swept shuttle-vs-body test can use the chord
// r0 -> r1 against the bodies' positions at t0 and t1 (environment ephemeris), or a cubic Hermite path from r, v.
struct ShuttleStepInfo {
    ShuttleId id = 0;
    std::size_t index = 0;
    double t0 = 0.0;
    double t1 = 0.0;
    Vec3 r0, v0, r1, v1;
};

// Hook for the collision logic (another module): called after every accepted step, in time order, on the thread that
// runs the shuttle, so it must be safe to use from several threads (write only to the events vector it is given, which
// belongs to that thread). Return false to stop: the shuttle is then marked dead (alive = 0), its state stays at t1
// and propagation ends. Push the ShuttleDestroyed event yourself (you know the contact time).
class IShuttleStepHook {
public:
    virtual ~IShuttleStepHook() = default;
    virtual bool onAcceptedStep(const ShuttleStepInfo& step, std::vector<Event>& events) = 0;
};

// Advance shuttle i across [t0, t1], one body step (plan 6.4). Per accepted step of length h:
//   thrust(h/2) -> drag(h/2) -> [DOPRI5 gravity step h] -> drag(h/2) -> thrust(h/2),
// with the reaction-wheel rotation (exact) advanced by h/2 in each half. Thrust is the exact rocket dv along the heading
// at the middle of the half step; drag is the exact update of plan 5.6 against air that rotates with each body.
// Steps end exactly at t1, at every controller tick k * environment.controllerTick at which the controller may change
// its command (IController::nextTick; a Coast shuttle has no such tick), and where a thrusting shuttle with finite fuel
// runs out of it (so the burn never stops inside a step, where the kick of the thrust operator would misplace the
// position by a * tau^2 / 2). Controllers are evaluated at the start of the
// step that begins at the tick, with a command held in between (stored in the Shuttles arrays). Events (FuelEmpty) go
// to `events`, a buffer owned by the calling thread. Nothing else is written outside element i of the store, so
// different shuttles can be advanced from different threads at the same time.
// A dead shuttle or t1 <= t0 does nothing. A non-finite state kills the shuttle (ShuttleDestroyed, by = 0).
// t0 must equal the time the shuttle was last left at (the previous call's t1); the bodies' ephemeris must cover [t0, t1].
// stats (optional) accumulates counters.
void propagateShuttle(Shuttles& s, std::size_t i, double t0, double t1, const ShuttleEnvironment& env,
                      std::vector<Event>& events, IShuttleStepHook* hook = nullptr, ShuttleStats* stats = nullptr);

} // namespace cosmos
