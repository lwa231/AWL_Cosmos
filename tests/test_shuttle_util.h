#pragma once
#include <doctest.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#include "core/bodies.h"
#include "core/controllers.h"
#include "core/kepler.h"
#include "core/shuttles.h"

// Helpers shared by the shuttle tests (M3).
namespace shuttle_test {

using namespace cosmos;

inline const double PI = 3.14159265358979323846;

// Domus-like numbers (plan section 13): mu = 3.767e12, R = 6.2e5, atmosphere rho0 = 1.22, H = 7000.
inline constexpr double kMuDomus = 3.767e12;
inline constexpr double kRDomus = 6.2e5;

inline BodyDesc bodyDesc(double mu, double radius, const Vec3& pos = {}, const Vec3& vel = {})
{
    BodyDesc d;
    d.mu = mu;
    d.radius = radius;
    d.position = pos;
    d.velocity = vel;
    return d;
}

// One static planet at the origin
inline Bodies singleBody(double mu, double radius = 0.0, const Vec3& pos = {})
{
    Bodies b;
    b.add(bodyDesc(mu, radius, pos));
    return b;
}

inline ShuttleDesc shuttleDesc(ShuttleId id, const Vec3& r, const Vec3& v, ControllerKind k = ControllerKind::Coast)
{
    ShuttleDesc d;
    d.id = id;
    d.position = r;
    d.velocity = v;
    d.controller = k;
    return d;
}

// Fixed command every tick (never changes): used to drive the rocket and rotation tests.
class ConstantController final : public IController {
public:
    ConstantController(double throttle, double turn) : cmd_{throttle, turn} {}
    ShuttleCommand update(const ShuttleObservation&, Rng&) override { return cmd_; }
    std::int64_t nextTick(const ShuttleObservation&) const override { return std::numeric_limits<std::int64_t>::max(); }

private:
    ShuttleCommand cmd_;
};

// Records every call; returns fn(tick) and asks to be called again at nextFn(tick) (default: every tick).
struct RecordedCall {
    std::int64_t tick;
    double t;
    ShuttleCommand held;
};

template <class CmdFn, class NextFn>
class ScriptedController final : public IController {
public:
    ScriptedController(CmdFn c, NextFn n) : cmdFn(c), nextFn(n) {}
    ShuttleCommand update(const ShuttleObservation& obs, Rng&) override
    {
        calls.push_back({obs.tick, obs.t_sim, obs.command});
        return cmdFn(obs.tick);
    }
    std::int64_t nextTick(const ShuttleObservation& obs) const override { return nextFn(obs.tick); }
    std::vector<RecordedCall> calls;

private:
    CmdFn cmdFn;
    NextFn nextFn;
};

template <class CmdFn, class NextFn>
ScriptedController<CmdFn, NextFn> scripted(CmdFn c, NextFn n) { return ScriptedController<CmdFn, NextFn>(c, n); }

// Records the accepted steps of a shuttle.
class StepRecorder final : public IShuttleStepHook {
public:
    bool onAcceptedStep(const ShuttleStepInfo& s, std::vector<Event>&) override
    {
        steps.push_back(s);
        return stopAfter == 0 || steps.size() < stopAfter;
    }
    std::vector<ShuttleStepInfo> steps;
    std::size_t stopAfter = 0; // 0 = never stop
};

// Position and velocity on a bound Kepler orbit (about a body at the origin) at time t after the epoch, M(0) = M0.
inline RelState keplerAt(double mu, double a, double e, double omega, double M0, double t)
{
    Elements el;
    el.a = a;
    el.e = e;
    el.omega = omega;
    const double n = std::sqrt(mu / (a * a * a));
    el.M = M0 + n * t;
    return elementsToState(el, mu);
}

inline double keplerPeriod(double mu, double a) { return 2.0 * PI * std::sqrt(a * a * a / mu); }

inline Vec3 shuttlePos(const Shuttles& s, std::size_t i) { return {s.state.x[i], s.state.y[i], s.state.z[i]}; }
inline Vec3 shuttleVel(const Shuttles& s, std::size_t i) { return {s.state.vx[i], s.state.vy[i], s.state.vz[i]}; }

inline bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

} // namespace shuttle_test
