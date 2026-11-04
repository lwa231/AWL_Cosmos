#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Controller ticks, held commands and the accepted-step hook (plan 4.6, 6.4, 7.2).

namespace {
struct FreeSpace {
    Bodies bodies;
    LinearEphemeris eph{bodies};
    ControllerSet controllers;
    ShuttleEnvironment env{bodies, eph, controllers};
    std::vector<Event> events;
    FreeSpace() { env.activeStepFraction = 0.0; } // let free-space steps grow: only ticks and interval ends bound them
};

double tickT(std::int64_t k) { return static_cast<double>(k) * 0.1; }
} // namespace

TEST_SUITE("controller ticks")
{
    TEST_CASE("ticks are evaluated at exactly k * 0.1 s; a tick at the end of an interval belongs to the next interval")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t) { return ShuttleCommand{0.0, 0.0}; }, [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {}, ControllerKind::Prograde)));

        propagateShuttle(s, 0, 0.0, 1.05, F.env, F.events);
        REQUIRE(ctrl.calls.size() == 11u); // ticks 0..10
        for (std::int64_t k = 0; k <= 10; ++k) {
            CHECK(ctrl.calls[static_cast<std::size_t>(k)].tick == k);
            CHECK(ctrl.calls[static_cast<std::size_t>(k)].t == tickT(k)); // exactly, not approximately
        }
        propagateShuttle(s, 0, 1.05, 2.0, F.env, F.events);
        REQUIRE(ctrl.calls.size() == 20u); // ticks 11..19; tick 20 sits at the end of the interval
        CHECK(ctrl.calls.back().tick == 19);
        propagateShuttle(s, 0, 2.0, 2.3, F.env, F.events);
        REQUIRE(ctrl.calls.size() == 23u); // ticks 20..22; tick 23 (2.3) is the end of this interval
        CHECK(ctrl.calls[20].tick == 20);
        CHECK(ctrl.calls[20].t == tickT(20));
        CHECK(ctrl.calls.back().tick == 22);
    }

    TEST_CASE("a custom tick period is honoured (0.25 s)")
    {
        FreeSpace F;
        F.env.controllerTick = 0.25;
        auto ctrl = scripted([](std::int64_t) { return ShuttleCommand{}; }, [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {}, ControllerKind::Prograde)));
        propagateShuttle(s, 0, 0.0, 1.1, F.env, F.events);
        REQUIRE(ctrl.calls.size() == 5u); // ticks 0..4 at 0, 0.25, 0.5, 0.75, 1.0
        for (std::size_t k = 0; k < 5; ++k) {
            CHECK(ctrl.calls[k].tick == static_cast<std::int64_t>(k));
            CHECK(ctrl.calls[k].t == 0.25 * static_cast<double>(k));
        }
    }

    TEST_CASE("a shuttle's first interval evaluates its controller at the start time, then at the next tick")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t) { return ShuttleCommand{}; }, [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {}, ControllerKind::Prograde)));
        propagateShuttle(s, 0, 0.37, 0.65, F.env, F.events);
        REQUIRE(ctrl.calls.size() == 4u); // 0.37 (tick 3), then 0.4, 0.5, 0.6
        CHECK(ctrl.calls[0].tick == 3);
        CHECK(ctrl.calls[0].t == 0.37);
        for (std::int64_t k = 4; k <= 6; ++k) {
            CHECK(ctrl.calls[static_cast<std::size_t>(k - 3)].tick == k);
            CHECK(ctrl.calls[static_cast<std::size_t>(k - 3)].t == tickT(k));
        }
    }

    TEST_CASE("the command is held between ticks: a burn on odd ticks gives exactly 10 x 0.1 s of thrust")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t k) { return ShuttleCommand{k % 2 == 1 ? 1.0 : 0.0, 0.0}; },
                             [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(1, {}, {}, ControllerKind::Prograde);
        d.heading = 0.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 2.0, F.env, F.events); // ticks 0..19; odd ticks 1,3,..,19 burn for 0.1 s each
        CHECK(s.state.vx[0] == doctest::Approx(10.0 * 0.1 * 12.0).epsilon(1e-12)); // a = 60 kN / 5000 kg = 12 m/s^2
        // each call saw the command of the previous tick as the held one
        for (std::size_t k = 1; k < ctrl.calls.size(); ++k) {
            CHECK(ctrl.calls[k].held.throttle == ((ctrl.calls[k].tick - 1) % 2 == 1 ? 1.0 : 0.0));
        }
        CHECK(ctrl.calls[0].held.throttle == 0.0);
    }

    TEST_CASE("a command that starts at tick 10 does not move the shuttle before t = 1.0 s")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t k) { return ShuttleCommand{k >= 10 ? 1.0 : 0.0, 0.0}; },
                             [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(1, {}, {}, ControllerKind::Prograde);
        d.heading = 0.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 1.0, F.env, F.events);
        CHECK(s.state.vx[0] == 0.0);           // bit-exact: the burn has not begun
        CHECK(s.throttle[0] == 0.0);
        propagateShuttle(s, 0, 1.0, 2.0, F.env, F.events);
        CHECK(s.state.vx[0] == doctest::Approx(12.0).epsilon(1e-12));
    }

    TEST_CASE("ticks that cannot change the command are skipped: not called, not step boundaries")
    {
        // Burn 0..5 s, coast 5..10 s, burn 10..: the controller only needs to be called every 50 ticks.
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t k) { return ShuttleCommand{(k / 50) % 2 == 0 ? 1.0 : 0.0, 0.0}; },
                             [](std::int64_t k) { return (k / 50 + 1) * 50; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(1, {}, {}, ControllerKind::Prograde);
        d.heading = 0.0;
        REQUIRE(s.add(d));
        StepRecorder rec;
        ShuttleStats st;
        propagateShuttle(s, 0, 0.0, 12.0, F.env, F.events, &rec, &st);
        REQUIRE(ctrl.calls.size() == 3u);
        CHECK(ctrl.calls[0].tick == 0);
        CHECK(ctrl.calls[1].tick == 50);
        CHECK(ctrl.calls[1].t == 5.0);
        CHECK(ctrl.calls[2].tick == 100);
        CHECK(ctrl.calls[2].t == 10.0);
        // dv: 5 s + 2 s of burn
        CHECK(s.state.vx[0] == doctest::Approx(7.0 * 12.0).epsilon(1e-12));
        // steps end exactly at the command changes and at the end of the interval ...
        auto endsAt = [&](double t) {
            return std::any_of(rec.steps.begin(), rec.steps.end(), [&](const ShuttleStepInfo& x) { return x.t1 == t; });
        };
        CHECK(endsAt(5.0));
        CHECK(endsAt(10.0));
        CHECK(endsAt(12.0));
        // ... and there are far fewer steps than the 120 ticks
        CHECK(st.acceptedSteps < 30);
        CHECK(st.controllerTicks == 3);
    }

    TEST_CASE("the tick schedule survives interval boundaries (state kept in the store)")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t k) { return ShuttleCommand{(k / 50) % 2 == 0 ? 1.0 : 0.0, 0.0}; },
                             [](std::int64_t k) { return (k / 50 + 1) * 50; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(1, {}, {}, ControllerKind::Prograde);
        d.heading = 0.0;
        REQUIRE(s.add(d));
        const double cuts[] = {0.0, 1.0, 4.9, 5.0, 5.05, 9.99, 10.0, 12.0};
        for (int k = 0; k < 7; ++k) propagateShuttle(s, 0, cuts[k], cuts[k + 1], F.env, F.events);
        REQUIRE(ctrl.calls.size() == 3u);
        CHECK(ctrl.calls[1].t == 5.0);
        CHECK(ctrl.calls[2].t == 10.0);
        CHECK(s.state.vx[0] == doctest::Approx(7.0 * 12.0).epsilon(1e-12));
        CHECK(s.nextTick[0] == 150);
    }

    TEST_CASE("a state-dependent controller (default: every tick) makes every tick a step boundary")
    {
        FreeSpace F;
        auto ctrl = scripted([](std::int64_t) { return ShuttleCommand{}; }, [](std::int64_t k) { return k + 1; });
        F.controllers.set(ControllerKind::Prograde, &ctrl);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {1.0, 0.0, 0.0}, ControllerKind::Prograde)));
        StepRecorder rec;
        propagateShuttle(s, 0, 0.0, 3.0, F.env, F.events, &rec);
        for (std::int64_t k = 1; k <= 29; ++k) {
            const double t = tickT(k);
            CHECK(std::any_of(rec.steps.begin(), rec.steps.end(), [&](const ShuttleStepInfo& x) { return x.t1 == t; }));
        }
        CHECK(rec.steps.size() >= 30u);
    }

    TEST_CASE("the observation carries the shuttle's state, mass and parent")
    {
        // One heavy body at (1e6, 0, 0) moving at (0, 10, 0); the shuttle is nearby.
        Bodies b;
        b.add(bodyDesc(kMuDomus, kRDomus, {1.0e6, 0.0, 0.0}, {0.0, 10.0, 0.0}));
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        ShuttleObservation seen;
        struct Spy final : IController {
            ShuttleObservation* out;
            explicit Spy(ShuttleObservation* o) : out(o) {}
            ShuttleCommand update(const ShuttleObservation& obs, Rng&) override { *out = obs; return {}; }
            std::int64_t nextTick(const ShuttleObservation&) const override { return std::numeric_limits<std::int64_t>::max(); }
        } spy(&seen);
        cs.set(ControllerKind::Prograde, &spy);
        Shuttles s;
        ShuttleDesc d = shuttleDesc(77, {1.0e6 + 8.0e5, 0.0, 0.0}, {0.0, 2300.0, 0.0}, ControllerKind::Prograde);
        d.heading = 0.4;
        d.spin = 0.1;
        d.fuelMass = 1500.0;
        REQUIRE(s.add(d));
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 0.5, env, ev);
        CHECK(seen.shuttleId == 77u);
        CHECK(seen.tick == 0);
        CHECK(seen.t_sim == 0.0);
        CHECK(seen.controllerTick_s == 0.1);
        CHECK(seen.position.x == 1.8e6);
        CHECK(seen.heading == 0.4);
        CHECK(seen.spin == 0.1);
        CHECK(seen.mass == 3500.0);
        CHECK(seen.dryMass == 2000.0);
        CHECK(seen.fuelMass == 1500.0);
        CHECK_FALSE(seen.infiniteFuel);
        CHECK(seen.maxThrust == 60000.0);
        CHECK(seen.exhaustVelocity == 3000.0);
        CHECK(seen.maxTorque == 5000.0);
        CHECK(seen.inertia == 1.0e4);
        CHECK(seen.parentId == b.ids[0]);
        CHECK(seen.parentMu == kMuDomus);
        CHECK(seen.relPosition.x == doctest::Approx(8.0e5).epsilon(1e-15));
        CHECK(seen.relVelocity.y == doctest::Approx(2290.0).epsilon(1e-15)); // relative to the moving parent
    }
}

TEST_SUITE("shuttle step hook")
{
    TEST_CASE("the hook sees every accepted step, in order, contiguous, covering the interval")
    {
        Bodies b = singleBody(kMuDomus, kRDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        const double r0 = 8.0e5;
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(3, {r0, 0.0, 0.0}, {0.0, std::sqrt(kMuDomus / r0), 0.0})));
        StepRecorder rec;
        ShuttleStats st;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 100.0, 3000.0, env, ev, &rec, &st);
        REQUIRE(rec.steps.size() == static_cast<std::size_t>(st.acceptedSteps));
        CHECK(rec.steps.front().t0 == 100.0);
        CHECK(rec.steps.back().t1 == 3000.0);
        for (std::size_t k = 0; k < rec.steps.size(); ++k) {
            const ShuttleStepInfo& x = rec.steps[k];
            CHECK(x.id == 3u);
            CHECK(x.index == 0u);
            CHECK(x.t1 > x.t0);
            if (k > 0) {
                const ShuttleStepInfo& p = rec.steps[k - 1];
                CHECK(x.t0 == p.t1);
                CHECK(x.r0.x == p.r1.x);
                CHECK(x.r0.y == p.r1.y);
                CHECK(x.v0.x == p.v1.x);
                CHECK(x.v0.y == p.v1.y);
            }
        }
        CHECK(rec.steps.back().r1.x == s.state.x[0]);
        CHECK(rec.steps.back().v1.y == s.state.vy[0]);
        // Each step is short on the orbit scale (a chord of a few degrees at most): the swept test on it is meaningful
        for (const ShuttleStepInfo& x : rec.steps) CHECK(norm(x.r1 - x.r0) < 0.1 * r0);
    }

    TEST_CASE("thrust and drag change v but never r inside a step, so r0 -> r1 is the gravity step's path")
    {
        FreeSpace F;
        ConstantController full(1.0, 0.0);
        F.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {}, ControllerKind::Prograde)));
        StepRecorder rec;
        propagateShuttle(s, 0, 0.0, 1.0, F.env, F.events, &rec);
        // free space: r1 = r0 + h * (mean velocity); the velocity grows from v0 to v1 along x
        for (const ShuttleStepInfo& x : rec.steps) {
            CHECK(x.v1.x >= x.v0.x);
            const double h = x.t1 - x.t0;
            CHECK(x.r1.x - x.r0.x >= h * x.v0.x - 1e-12);
            CHECK(x.r1.x - x.r0.x <= h * x.v1.x + 1e-12);
        }
    }

    TEST_CASE("returning false kills the shuttle at that step's end and stops; a dead shuttle is skipped later")
    {
        Bodies b = singleBody(kMuDomus, kRDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(3, {8.0e5, 0.0, 0.0}, {0.0, 2000.0, 0.0})));
        REQUIRE(s.add(shuttleDesc(4, {8.0e5, 0.0, 0.0}, {0.0, 2000.0, 0.0})));
        StepRecorder rec;
        rec.stopAfter = 3;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 5000.0, env, ev, &rec);
        REQUIRE(rec.steps.size() == 3u);
        CHECK(s.alive[0] == 0);
        CHECK(s.alive[1] == 1);
        CHECK(s.aliveCount() == 1u);
        CHECK(s.state.x[0] == rec.steps[2].r1.x);
        CHECK(s.state.vy[0] == rec.steps[2].v1.y);
        CHECK(ev.empty()); // the hook is responsible for its own events
        // dead: nothing happens
        const double x = s.state.x[0];
        StepRecorder rec2;
        propagateShuttle(s, 0, 5000.0, 6000.0, env, ev, &rec2);
        CHECK(rec2.steps.empty());
        CHECK(s.state.x[0] == x);
        // the hook may emit events
        struct Emitter final : IShuttleStepHook {
            bool onAcceptedStep(const ShuttleStepInfo& st, std::vector<Event>& events) override
            {
                events.push_back(makeShuttleDestroyed(st.t1, st.id, 1));
                return false;
            }
        } emitter;
        propagateShuttle(s, 1, 0.0, 5000.0, env, ev, &emitter);
        REQUIRE(ev.size() == 1u);
        CHECK(ev[0].type == EventType::ShuttleDestroyed);
        CHECK(ev[0].id == 4u);
        CHECK(ev[0].other == 1u);
        CHECK(s.alive[1] == 0);
    }

    TEST_CASE("an empty interval does nothing")
    {
        FreeSpace F;
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0})));
        StepRecorder rec;
        propagateShuttle(s, 0, 5.0, 5.0, F.env, F.events, &rec);
        propagateShuttle(s, 0, 5.0, 4.0, F.env, F.events, &rec);
        CHECK(rec.steps.empty());
        CHECK(s.state.x[0] == 1.0);
        CHECK(s.nextTick[0] == kTickUnset);
    }

    TEST_CASE("a shuttle that reaches the singularity of a point mass is destroyed instead of poisoning the run")
    {
        Bodies b = singleBody(kMuDomus, 0.0); // point mass
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(9, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}))); // exactly at the centre: infinite field
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 10.0, env, ev);
        CHECK(s.alive[0] == 0);
        REQUIRE(ev.size() == 1u);
        CHECK(ev[0].type == EventType::ShuttleDestroyed);
        CHECK(ev[0].id == 9u);
        CHECK(ev[0].other == 0u);
        CHECK(std::isfinite(s.state.x[0]));
    }
}
