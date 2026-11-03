#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include "core/bodies.h"
#include "core/rng.h"
#include "core/vec3.h"

namespace cosmos {

// Shuttle controllers (plan sections 7.2, 7.3). A controller is evaluated on fixed sim-time ticks k * controllerTick_s
// (never per frame) and its command is held until the next evaluation. The interface is the future ML plug-in point:
// a Python link is just another IController.

// What a controller may set (plan 5.7). Values outside the ranges are clamped by the propagator.
struct ShuttleCommand {
    double throttle = 0.0; // [0, 1]
    double turn = 0.0;     // [-1, 1], reaction-wheel torque as a fraction of the maximum
};

inline bool operator==(const ShuttleCommand& a, const ShuttleCommand& b) { return a.throttle == b.throttle && a.turn == b.turn; }
inline bool operator!=(const ShuttleCommand& a, const ShuttleCommand& b) { return !(a == b); }

// Everything a controller sees at a tick. Plain data, no pointers, so it can be serialised for a remote controller.
struct ShuttleObservation {
    std::uint64_t shuttleId = 0;
    std::int64_t tick = 0;          // tick index k; the tick's nominal time is k * controllerTick_s
    double t_sim = 0.0;             // sim time at which the observation is taken (= the tick time, except for a shuttle's first tick)
    double controllerTick_s = 0.1;

    Vec3 position;                  // absolute, in the frame of the bodies
    Vec3 velocity;
    double heading = 0.0;           // theta, rad, counter-clockwise from +x
    double spin = 0.0;              // omega_s, rad/s

    double mass = 0.0;              // dry + fuel, kg
    double dryMass = 0.0;
    double fuelMass = 0.0;
    bool infiniteFuel = false;
    double maxThrust = 0.0;         // N
    double exhaustVelocity = 0.0;   // m/s
    double maxTorque = 0.0;         // N m
    double inertia = 1.0;           // kg m^2

    BodyId parentId = 0;            // 0 = the system barycentre (or no body)
    double parentMu = 0.0;          // m^3/s^2 of the parent node
    Vec3 relPosition;               // shuttle minus parent
    Vec3 relVelocity;

    ShuttleCommand command;         // the command currently held (the result of the previous tick)
};

class IController {
public:
    virtual ~IController() = default;

    // Called on a tick. rng is the generator for (shuttleId, tick) of rng.h.
    virtual ShuttleCommand update(const ShuttleObservation& obs, Rng& rng) = 0;

    // First tick index > obs.tick at which update() must be called again. Ticks before it are skipped: update() is not
    // called and, because the command cannot change there, they are not step boundaries of the shuttle integrator
    // (plan 6.4: a tick that leaves the command unchanged is not a step boundary). The default asks for every tick.
    // The answer may depend only on obs (computed right after update(obs)), never on hidden mutable state.
    virtual std::int64_t nextTick(const ShuttleObservation& obs) const { return obs.tick + 1; }
};

enum class ControllerKind : std::uint8_t { Coast, Prograde, Retrograde, Wander };

const char* controllerName(ControllerKind k);
// Case-insensitive; accepts "Coast", "Prograde", "Retrograde", "Wander". Returns false for anything else.
bool parseControllerKind(const std::string& name, ControllerKind& out);

// Wraps an angle to (-pi, pi].
double wrapAngle(double a);

// ---- Pointing (plan 7.3): turn = clamp(Kp * angleError - Kd * omega_s, -1, 1) --------------------------------------
// The gains are written for the default reaction-wheel authority alpha0 = tau_max / I = 5 kN m / 1e4 kg m^2 = 0.5 rad/s^2
// (plan 7.1) and rescaled by alpha0 / alpha for other shuttles, so the linear closed loop
//     theta'' = alpha (Kp e - Kd omega)   ->   omega_n = sqrt(alpha0 Kp) = 1 rad/s,  zeta = alpha0 Kd / (2 omega_n) = 0.9
// is the same for every shuttle. omega_n * tick = 0.1 keeps the 10 Hz sample-and-hold loop well inside its stability margin;
// the tests measure the settling from a 180 degree error.
inline constexpr double kPointingKp = 2.0;            // turn per rad of error
inline constexpr double kPointingKd = 3.6;            // turn per rad/s of spin
inline constexpr double kPointingAuthority0 = 0.5;    // rad/s^2

// Turn command that points heading at target. authority = tau_max / I (rad/s^2); 0 gives 0.
double pointingTurn(double targetHeading, double heading, double spin, double authority);

// Throttle is on only while the heading is within this angle of the wanted direction (rad, about 20 degrees:
// cos = 0.94, so a burn loses at most 6% of its thrust to misalignment).
inline constexpr double kBurnAlignmentLimit = 0.35;

// ---- Built-in autopilots (plan 7.3). All stateless, so one instance can serve every shuttle and every thread. -------

// Throttle 0, no turning. Never changes, so it asks to be called once only.
class CoastController final : public IController {
public:
    ShuttleCommand update(const ShuttleObservation& obs, Rng& rng) override;
    std::int64_t nextTick(const ShuttleObservation&) const override { return std::numeric_limits<std::int64_t>::max(); }
};

// Points along (sign = +1) or against (sign = -1) the velocity relative to the parent and burns at full throttle once
// aligned. With no relative motion it holds its heading.
class BurnController : public IController {
public:
    explicit BurnController(double sign) : sign_(sign) {}
    ShuttleCommand update(const ShuttleObservation& obs, Rng& rng) override;

private:
    double sign_;
};

class ProgradeController final : public BurnController {
public:
    ProgradeController() : BurnController(+1.0) {}
};

class RetrogradeController final : public BurnController {
public:
    RetrogradeController() : BurnController(-1.0) {}
};

// Random short burns and turns (plan 7.3). Time is cut into segments of segmentTicks ticks (2 s by default); each segment
// draws its command from the shuttle's own counter RNG Rng::forTick(id, first tick of the segment). So the command at
// any tick is a pure function of (shuttle id, tick), independent of the state, of the thread and of what was evaluated
// before. The command is constant within a segment, so only the segment starts are ticks.
struct WanderParams {
    std::int64_t segmentTicks = 20;
    double burnProbability = 0.4;   // chance that a segment burns
    double minThrottle = 0.3;       // burn throttle is uniform in [minThrottle, 1]
    double turnProbability = 0.7;   // chance that a segment turns; the turn is uniform in [-1, 1]
};

class WanderController final : public IController {
public:
    explicit WanderController(const WanderParams& p = WanderParams()) : params_(p) {}
    ShuttleCommand update(const ShuttleObservation& obs, Rng& rng) override;
    std::int64_t nextTick(const ShuttleObservation& obs) const override;

    // The command of the segment that contains this tick. update() returns exactly this.
    ShuttleCommand commandFor(std::uint64_t shuttleId, std::int64_t tick) const;

private:
    WanderParams params_;
};

// The four built-in autopilots, selectable by ControllerKind; set() replaces one (for example with a test double or an
// ML link). Not copyable (the defaults point into the object).
class ControllerSet {
public:
    ControllerSet();
    ControllerSet(const ControllerSet&) = delete;
    ControllerSet& operator=(const ControllerSet&) = delete;

    IController* get(ControllerKind k) const { return byKind_[static_cast<int>(k)]; }
    void set(ControllerKind k, IController* c) { byKind_[static_cast<int>(k)] = c; }

private:
    CoastController coast_;
    ProgradeController prograde_;
    RetrogradeController retrograde_;
    WanderController wander_;
    IController* byKind_[4];
};

} // namespace cosmos
