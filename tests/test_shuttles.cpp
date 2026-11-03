#include <doctest.h>
#include <cmath>
#include <limits>
#include "core/scene.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Shuttle store, rocket equation, reaction wheels, fuel, and the split-operator thrust (plan 5.7, 6.4, 7.1, 14 "Rocket").

namespace {
// Free space: no bodies at all
struct FreeSpace {
    Bodies bodies;
    LinearEphemeris eph{bodies};
    ControllerSet controllers;
    ShuttleEnvironment env{bodies, eph, controllers};
    std::vector<Event> events;
};
} // namespace

TEST_SUITE("shuttles store")
{
    TEST_CASE("ShuttleDesc defaults are the table of plan 7.1")
    {
        const ShuttleDesc d;
        CHECK(d.dryMass == 2000.0);
        CHECK(d.fuelMass == 3000.0);
        CHECK(d.maxThrust == 60000.0);
        CHECK(d.exhaustVelocity == 3000.0);
        CHECK(d.cdA == 2.0);
        CHECK(d.radius == 2.0);
        CHECK(d.maxTorque == 5000.0);
        CHECK(d.inertia == 1.0e4);
        Shuttles s;
        CHECK_FALSE(s.infiniteFuel); // the global switch is off by default
    }

    TEST_CASE("add: fields land in the arrays, ids are permanent and never reused")
    {
        Shuttles s;
        ShuttleDesc a = shuttleDesc(7, {1.0, 2.0, 0.0}, {3.0, 4.0, 0.0}, ControllerKind::Wander);
        a.name = "Alpha";
        a.heading = 0.5;
        a.spin = -0.25;
        REQUIRE(s.add(a));
        REQUIRE(s.add(shuttleDesc(9, {0, 0, 0}, {0, 0, 0})));
        REQUIRE(s.consistent());
        CHECK(s.size() == 2u);
        CHECK(s.aliveCount() == 2u);
        const std::size_t i = *s.find(7);
        CHECK(i == 0u);
        CHECK(s.name[i] == "Alpha");
        CHECK(s.controller[i] == ControllerKind::Wander);
        CHECK(shuttlePos(s, i).y == 2.0);
        CHECK(shuttleVel(s, i).x == 3.0);
        CHECK(s.heading[i] == 0.5);
        CHECK(s.spin[i] == -0.25);
        CHECK(s.mass(i) == 5000.0);
        CHECK(s.throttle[i] == 0.0);
        CHECK(s.turn[i] == 0.0);
        CHECK(s.alive[i] == 1);
        CHECK(s.nextTick[i] == kTickUnset);
        CHECK_FALSE(s.find(8).has_value());

        // duplicates and id 0 refused, nothing added
        CHECK_FALSE(s.add(shuttleDesc(7, {}, {})));
        CHECK_FALSE(s.add(shuttleDesc(0, {}, {})));
        CHECK(s.size() == 2u);

        // after removal the id stays retired
        s.kill(0);
        CHECK(s.aliveCount() == 1u);
        s.compact();
        REQUIRE(s.consistent());
        CHECK(s.size() == 1u);
        CHECK_FALSE(s.find(7).has_value());
        CHECK(*s.find(9) == 0u);
        CHECK_FALSE(s.add(shuttleDesc(7, {}, {})));
        CHECK(s.add(shuttleDesc(10, {}, {})));
        CHECK(*s.find(10) == 1u);
    }

    TEST_CASE("add refuses impossible shuttles")
    {
        Shuttles s;
        ShuttleDesc d = shuttleDesc(1, {}, {});
        d.dryMass = 0.0;
        CHECK_FALSE(s.add(d));
        d = shuttleDesc(1, {}, {});
        d.fuelMass = -1.0;
        CHECK_FALSE(s.add(d));
        d = shuttleDesc(1, {}, {});
        d.inertia = 0.0;
        CHECK_FALSE(s.add(d));
        d = shuttleDesc(1, {}, {});
        d.maxThrust = std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(s.add(d));
        d = shuttleDesc(1, {std::numeric_limits<double>::infinity(), 0, 0}, {});
        CHECK_FALSE(s.add(d));
        CHECK(s.size() == 0u);
        CHECK(s.add(shuttleDesc(1, {}, {}))); // a failed add does not burn the id
    }

    TEST_CASE("compact keeps the order and all columns of the survivors")
    {
        Shuttles s;
        for (ShuttleId id = 1; id <= 6; ++id) {
            ShuttleDesc d = shuttleDesc(id, {double(id), 0, 0}, {0, double(10 * id), 0}, ControllerKind::Prograde);
            d.fuelMass = 100.0 * id;
            d.name = "s" + std::to_string(id);
            REQUIRE(s.add(d));
        }
        s.throttle[3] = 0.75;
        s.nextTick[4] = 42;
        s.kill(1);
        s.kill(2);
        s.kill(5);
        s.compact();
        REQUIRE(s.consistent());
        REQUIRE(s.size() == 3u);
        CHECK(s.ids == std::vector<ShuttleId>{1, 4, 5});
        CHECK(s.name[1] == "s4");
        CHECK(s.state.x[1] == 4.0);
        CHECK(s.state.vy[2] == 50.0);
        CHECK(s.fuelMass[1] == 400.0);
        CHECK(s.throttle[1] == 0.75);
        CHECK(s.nextTick[2] == 42);
        CHECK(*s.find(5) == 2u);
    }

    TEST_CASE("built from a resolved scene shuttle: parameters, state, controller by name, heading along the velocity")
    {
        Scene sc;
        SceneBody planet;
        planet.id = 1;
        planet.name = "P";
        planet.mu = kMuDomus;
        planet.radius = kRDomus;
        sc.bodies.push_back(planet);
        SceneShuttle sh;
        sh.id = 10;
        sh.name = "Orbiter";
        sh.controller = "retrograde";
        sh.fuelMass = 1234.0;
        sh.maxThrust = 5e4;
        SceneOrbit orb;
        orb.parent = 1;
        orb.a = 8.0e5;
        orb.e = 0.0;
        sh.orbit = orb;
        sc.shuttles.push_back(sh);
        SceneShuttle bad = sh;
        bad.id = 11;
        bad.controller = "Hover";
        sc.shuttles.push_back(bad);

        ResolvedScene res;
        std::string err;
        REQUIRE_MESSAGE(resolveScene(sc, res, err), err);
        REQUIRE(res.shuttles.size() == 2u);

        Shuttles s;
        REQUIRE(s.addResolved(res.shuttles[0]));
        CHECK_FALSE(s.addResolved(res.shuttles[1])); // unknown controller name
        REQUIRE(s.size() == 1u);
        CHECK(s.ids[0] == 10u);
        CHECK(s.name[0] == "Orbiter");
        CHECK(s.controller[0] == ControllerKind::Retrograde);
        CHECK(s.fuelMass[0] == 1234.0);
        CHECK(s.maxThrust[0] == 5e4);
        CHECK(s.dryMass[0] == 2000.0);
        CHECK(norm(shuttlePos(s, 0)) == doctest::Approx(8.0e5).epsilon(1e-12));
        CHECK(norm(shuttleVel(s, 0)) == doctest::Approx(std::sqrt(kMuDomus / 8.0e5)).epsilon(1e-12));
        const Vec3 v = shuttleVel(s, 0);
        CHECK(s.heading[0] == doctest::Approx(std::atan2(v.y, v.x)).epsilon(1e-15));
        CHECK(s.spin[0] == 0.0);
    }
}

TEST_SUITE("rocket")
{
    TEST_CASE("rocketBurn: full burn gives dv = ve ln(m0 / m_dry) to 1e-9")
    {
        const BurnResult b = rocketBurn(2000.0, 3000.0, 1.0, 60000.0, 3000.0, 1000.0, false);
        CHECK(b.emptied);
        CHECK(b.burnTime == doctest::Approx(150.0).epsilon(1e-15)); // 3000 kg at 20 kg/s
        CHECK(b.fuelUsed == 3000.0);
        CHECK(b.dv == doctest::Approx(3000.0 * std::log(2.5)).epsilon(1e-9));
        CHECK(b.dv == doctest::Approx(2748.8).epsilon(1e-4)); // the plan's "about 2.75 km/s"
    }

    TEST_CASE("rocketBurn: partial burns follow the rocket equation and add up (Tsiolkovsky is additive)")
    {
        const double dry = 2000.0, fuel = 3000.0, T = 60000.0, ve = 3000.0;
        const BurnResult b1 = rocketBurn(dry, fuel, 1.0, T, ve, 40.0, false);
        CHECK_FALSE(b1.emptied);
        CHECK(b1.fuelUsed == doctest::Approx(800.0).epsilon(1e-15));
        CHECK(b1.dv == doctest::Approx(ve * std::log(5000.0 / 4200.0)).epsilon(1e-13));
        // second burn of 60 s from the remaining fuel; total 100 s must equal one burn of 100 s
        const BurnResult b2 = rocketBurn(dry, fuel - b1.fuelUsed, 1.0, T, ve, 60.0, false);
        const BurnResult all = rocketBurn(dry, fuel, 1.0, T, ve, 100.0, false);
        CHECK(b1.dv + b2.dv == doctest::Approx(all.dv).epsilon(1e-13));
        // half throttle: half the mass flow, same exhaust velocity
        const BurnResult half = rocketBurn(dry, fuel, 0.5, T, ve, 80.0, false);
        CHECK(half.fuelUsed == doctest::Approx(800.0).epsilon(1e-15));
        CHECK(half.dv == doctest::Approx(b1.dv).epsilon(1e-13));
        // a very short burn is not lost to rounding: dv = T t / m0 to first order
        const BurnResult tiny = rocketBurn(dry, fuel, 1.0, T, ve, 1e-9, false);
        CHECK(tiny.dv == doctest::Approx(T * 1e-9 / 5000.0).epsilon(1e-6));
    }

    TEST_CASE("rocketBurn: infinite fuel keeps the mass and gives dv = T t / m")
    {
        const BurnResult b = rocketBurn(2000.0, 3000.0, 1.0, 60000.0, 3000.0, 400.0, true);
        CHECK(b.dv == doctest::Approx(60000.0 * 400.0 / 5000.0).epsilon(1e-15));
        CHECK(b.fuelUsed == 0.0);
        CHECK_FALSE(b.emptied);
        CHECK(b.burnTime == 400.0);
        // also with an empty tank
        CHECK(rocketBurn(2000.0, 0.0, 1.0, 60000.0, 3000.0, 10.0, true).dv == doctest::Approx(60000.0 * 10.0 / 2000.0).epsilon(1e-15));
    }

    TEST_CASE("rocketBurn: nothing happens without throttle, fuel, thrust or time; throttle is clamped")
    {
        CHECK(rocketBurn(2000.0, 3000.0, 0.0, 60000.0, 3000.0, 10.0, false).dv == 0.0);
        CHECK(rocketBurn(2000.0, 3000.0, -1.0, 60000.0, 3000.0, 10.0, false).dv == 0.0);
        CHECK(rocketBurn(2000.0, 0.0, 1.0, 60000.0, 3000.0, 10.0, false).dv == 0.0);
        CHECK(rocketBurn(2000.0, 3000.0, 1.0, 0.0, 3000.0, 10.0, false).dv == 0.0);
        CHECK(rocketBurn(2000.0, 3000.0, 1.0, 60000.0, 0.0, 10.0, false).dv == 0.0);
        CHECK(rocketBurn(2000.0, 3000.0, 1.0, 60000.0, 3000.0, 0.0, false).dv == 0.0);
        const BurnResult over = rocketBurn(2000.0, 3000.0, 7.0, 60000.0, 3000.0, 10.0, false);
        const BurnResult one = rocketBurn(2000.0, 3000.0, 1.0, 60000.0, 3000.0, 10.0, false);
        CHECK(over.dv == one.dv);
    }

    TEST_CASE("full burn in free space through propagateShuttle: dv = ve ln(m0 / m_dry) to 1e-9, along the heading")
    {
        FreeSpace F;
        ConstantController full(1.0, 0.0);
        F.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        ShuttleDesc d = shuttleDesc(5, {10.0, 20.0, 0.0}, {100.0, -50.0, 0.0}, ControllerKind::Prograde);
        d.heading = 0.6;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 400.0, F.env, F.events);

        const double dv = 3000.0 * std::log(2.5);
        const Vec3 dvec = shuttleVel(s, 0) - Vec3{100.0, -50.0, 0.0};
        CHECK(norm(dvec) == doctest::Approx(dv).epsilon(1e-9));
        CHECK(dvec.x / norm(dvec) == doctest::Approx(std::cos(0.6)).epsilon(1e-12));
        CHECK(dvec.y / norm(dvec) == doctest::Approx(std::sin(0.6)).epsilon(1e-12));
        CHECK(dvec.z == 0.0);
        CHECK(s.fuelMass[0] == 0.0);
        CHECK(s.mass(0) == 2000.0);
        // position: coasts after burnout. Check against the exact integral of the burn x(t): finite-burn distance
        // s_burn = ve * (t_b - (m_dry... )) is checked indirectly via the velocity; here the end position is consistent
        // with a mean velocity between v0 and v_final.
        const Vec3 moved = shuttlePos(s, 0) - Vec3{10.0, 20.0, 0.0};
        CHECK(norm(moved) > 400.0 * 50.0);
    }

    TEST_CASE("the burn ends mid-run: exactly one FuelEmpty at m_fuel / mdot, then the throttle is forced to zero")
    {
        FreeSpace F;
        ConstantController full(1.0, 0.0);
        F.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        ShuttleDesc d = shuttleDesc(5, {}, {}, ControllerKind::Prograde);
        d.heading = 0.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 100.0, F.env, F.events);
        CHECK(F.events.empty());
        CHECK(s.fuelMass[0] == doctest::Approx(3000.0 - 2000.0).epsilon(1e-9)); // 100 s * 20 kg/s used
        CHECK(s.fuelEmptyReported[0] == 0);
        propagateShuttle(s, 0, 100.0, 200.0, F.env, F.events);
        REQUIRE(F.events.size() == 1u);
        CHECK(F.events[0].type == EventType::FuelEmpty);
        CHECK(F.events[0].id == 5u);
        CHECK(F.events[0].t == doctest::Approx(150.0).epsilon(1e-9));
        CHECK(s.fuelEmptyReported[0] == 1);
        const double vAfter = s.state.vx[0];
        CHECK(vAfter == doctest::Approx(3000.0 * std::log(2.5)).epsilon(1e-9));
        // The controller still asks for full throttle (the held command), the engine does not run and nothing is reported again
        CHECK(s.throttle[0] == 1.0);
        propagateShuttle(s, 0, 200.0, 500.0, F.env, F.events);
        CHECK(F.events.size() == 1u);
        CHECK(s.state.vx[0] == vAfter);
        CHECK(s.fuelMass[0] == 0.0);
    }

    TEST_CASE("a shuttle that starts with an empty tank reports FuelEmpty once, at its first thrust, and never accelerates")
    {
        FreeSpace F;
        ConstantController full(1.0, 0.0);
        F.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        ShuttleDesc d = shuttleDesc(5, {}, {1.0, 2.0, 0.0}, ControllerKind::Prograde);
        d.fuelMass = 0.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 3.0, 10.0, F.env, F.events);
        propagateShuttle(s, 0, 10.0, 20.0, F.env, F.events);
        REQUIRE(F.events.size() == 1u);
        CHECK(F.events[0].type == EventType::FuelEmpty);
        CHECK(F.events[0].t == 3.0);
        CHECK(s.state.vx[0] == 1.0);
        CHECK(s.state.vy[0] == 2.0);
    }

    TEST_CASE("infinite fuel (global switch): no mass change, no FuelEmpty, dv = T t / m, even beyond the tank")
    {
        FreeSpace F;
        ConstantController full(1.0, 0.0);
        F.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(5, {}, {}, ControllerKind::Prograde);
        d.heading = PI / 2.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 300.0, F.env, F.events); // 2x the burn time of the tank
        CHECK(F.events.empty());
        CHECK(s.fuelMass[0] == 3000.0);
        CHECK(s.mass(0) == 5000.0);
        CHECK(s.state.vy[0] == doctest::Approx(60000.0 * 300.0 / 5000.0).epsilon(1e-9));
        CHECK(std::fabs(s.state.vx[0]) < 1e-9);
        // An empty tank does not matter either
        Shuttles t;
        t.infiniteFuel = true;
        ShuttleDesc e = shuttleDesc(6, {}, {}, ControllerKind::Prograde);
        e.fuelMass = 0.0;
        REQUIRE(t.add(e));
        propagateShuttle(t, 0, 0.0, 10.0, F.env, F.events);
        CHECK(F.events.empty());
        CHECK(t.state.vx[0] == doctest::Approx(60000.0 * 10.0 / 2000.0).epsilon(1e-9));
    }

    TEST_CASE("half throttle: half the mass flow, dv from the same rocket equation")
    {
        FreeSpace F;
        ConstantController half(0.5, 0.0);
        F.controllers.set(ControllerKind::Prograde, &half);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(5, {}, {}, ControllerKind::Prograde)));
        propagateShuttle(s, 0, 0.0, 100.0, F.env, F.events);
        CHECK(3000.0 - s.fuelMass[0] == doctest::Approx(0.5 * 20.0 * 100.0).epsilon(1e-9));
        CHECK(s.state.vx[0] == doctest::Approx(3000.0 * std::log(5000.0 / 4000.0)).epsilon(1e-9));
    }

    TEST_CASE("commands are clamped to throttle [0, 1], turn [-1, 1], and NaN means zero")
    {
        FreeSpace F;
        ConstantController wild(5.0, -7.0);
        ConstantController nan(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity());
        F.controllers.set(ControllerKind::Prograde, &wild);
        F.controllers.set(ControllerKind::Retrograde, &nan);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {}, {}, ControllerKind::Prograde)));
        REQUIRE(s.add(shuttleDesc(2, {}, {}, ControllerKind::Retrograde)));
        propagateShuttle(s, 0, 0.0, 1.0, F.env, F.events);
        propagateShuttle(s, 1, 0.0, 1.0, F.env, F.events);
        CHECK(s.throttle[0] == 1.0);
        CHECK(s.turn[0] == -1.0);
        CHECK(s.throttle[1] == 0.0);
        CHECK(s.turn[1] == 0.0);
        CHECK(s.state.vx[1] == 0.0);
        // full clamped torque for 1 s: alpha = -5000 / 1e4 = -0.5
        CHECK(s.spin[0] == doctest::Approx(-0.5).epsilon(1e-14));
    }
}

TEST_SUITE("reaction wheels")
{
    TEST_CASE("advanceRotation: omega linear, theta quadratic, exact")
    {
        const Rotation r = advanceRotation(0.2, 0.3, -0.4, 5.0);
        CHECK(r.omega == doctest::Approx(0.3 - 0.4 * 5.0).epsilon(1e-15));
        CHECK(r.theta == doctest::Approx(0.2 + 0.3 * 5.0 - 0.5 * 0.4 * 25.0).epsilon(1e-15));
        // composition: two halves equal the whole
        const Rotation h1 = advanceRotation(0.2, 0.3, -0.4, 2.0);
        const Rotation h2 = advanceRotation(h1.theta, h1.omega, -0.4, 3.0);
        CHECK(h2.theta == doctest::Approx(r.theta).epsilon(1e-14));
        CHECK(h2.omega == doctest::Approx(r.omega).epsilon(1e-14));
    }

    TEST_CASE("constant torque through propagateShuttle: omega = alpha t, theta = theta0 + omega0 t + alpha t^2 / 2 to 1e-12")
    {
        FreeSpace F;
        ConstantController turn(0.0, 0.6);
        F.controllers.set(ControllerKind::Prograde, &turn);
        Shuttles s;
        ShuttleDesc d = shuttleDesc(5, {}, {1.0, 0.0, 0.0}, ControllerKind::Prograde);
        d.heading = 0.2;
        d.spin = -0.3;
        d.maxTorque = 8000.0;
        d.inertia = 2.0e4; // alpha = 8000 * 0.6 / 2e4 = 0.24 rad/s^2
        REQUIRE(s.add(d));
        const double alpha = 0.24, T = 3.7;
        propagateShuttle(s, 0, 0.0, T, F.env, F.events);
        CHECK(s.spin[0] == doctest::Approx(-0.3 + alpha * T).epsilon(1e-12));
        CHECK(s.heading[0] == doctest::Approx(0.2 - 0.3 * T + 0.5 * alpha * T * T).epsilon(1e-12));
        CHECK(s.fuelMass[0] == 3000.0); // turning costs no fuel
        // velocity untouched (torque only turns)
        CHECK(s.state.vx[0] == 1.0);

        // A long run wraps the heading into (-pi, pi]; compare modulo 2 pi
        Shuttles long_;
        d.id = 6;
        d.heading = 0.2;
        d.spin = -0.3;
        REQUIRE(long_.add(d));
        const double T2 = 60.0;
        propagateShuttle(long_, 0, 0.0, T2, F.env, F.events);
        const double expect = 0.2 - 0.3 * T2 + 0.5 * alpha * T2 * T2;
        CHECK(long_.spin[0] == doctest::Approx(-0.3 + alpha * T2).epsilon(1e-12));
        CHECK(long_.heading[0] > -PI - 1e-12);
        CHECK(long_.heading[0] <= PI + 1e-12);
        CHECK(std::fabs(wrapAngle(long_.heading[0] - expect)) < 1e-11);
    }

    TEST_CASE("without torque the spin is constant and the heading advances linearly")
    {
        FreeSpace F;
        Shuttles s;
        ShuttleDesc d = shuttleDesc(5, {}, {}, ControllerKind::Coast);
        d.spin = 0.7;
        d.heading = -1.0;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, 2.0, F.env, F.events);
        CHECK(s.spin[0] == 0.7);
        CHECK(s.heading[0] == doctest::Approx(-1.0 + 1.4).epsilon(1e-14));
    }

    TEST_CASE("thrust direction follows the turning heading: second-order accurate in the step (spinning at 0.5 rad/s)")
    {
        // Free space, infinite fuel, constant 12 m/s^2. The heading is constant-rate, so the exact velocity is
        // v = (a / w) (sin(th0 + w t) - sin th0, cos th0 - cos(th0 + w t)). The thrust of each half step points along the
        // heading at the middle of the half step, which loses (w d)^2 / 24 of every impulse: the error is O(h^2).
        // The step is set through activeStepFraction (dynamical time = 1 s without a body); the heading limit is off.
        const double w = 0.5, th0 = 0.4, a = 12.0, T = 20.0;
        auto errorFor = [&](double h) {
            FreeSpace F;
            F.env.activeStepFraction = h;
            F.env.maxHeadingChangePerStep = 0.0;
            ConstantController full(1.0, 0.0);
            F.controllers.set(ControllerKind::Prograde, &full);
            Shuttles s;
            s.infiniteFuel = true;
            ShuttleDesc d = shuttleDesc(5, {}, {}, ControllerKind::Prograde);
            d.heading = th0;
            d.spin = w;
            REQUIRE(s.add(d));
            propagateShuttle(s, 0, 0.0, T, F.env, F.events);
            const Vec3 ex{a / w * (std::sin(th0 + w * T) - std::sin(th0)), a / w * (std::cos(th0) - std::cos(th0 + w * T)), 0.0};
            return norm(shuttleVel(s, 0) - ex);
        };
        const double e1 = errorFor(0.8), e2 = errorFor(0.4), e3 = errorFor(0.2), e4 = errorFor(0.1);
        // Measured ratios 4.04, 4.04, 4.01 (the leading error (w d)^2 / 24 is quadratic)
        CHECK(e1 / e2 > 3.8);
        CHECK(e1 / e2 < 4.2);
        CHECK(e2 / e3 > 3.8);
        CHECK(e2 / e3 < 4.2);
        CHECK(e3 / e4 > 3.8);
        CHECK(e3 / e4 < 4.2);
        // The default heading limit (0.1 rad per step) keeps the error of this fast-spinning case below 1e-4 of dv
        FreeSpace G;
        ConstantController full(1.0, 0.0);
        G.controllers.set(ControllerKind::Prograde, &full);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(5, {}, {}, ControllerKind::Prograde);
        d.heading = th0;
        d.spin = w;
        REQUIRE(s.add(d));
        propagateShuttle(s, 0, 0.0, T, G.env, G.events);
        const Vec3 ex{a / w * (std::sin(th0 + w * T) - std::sin(th0)), a / w * (std::cos(th0) - std::cos(th0 + w * T)), 0.0};
        CHECK(norm(shuttleVel(s, 0) - ex) < 1e-4 * a * T);
    }

    TEST_CASE("thrust while turning under torque: heading limit keeps the direction error small")
    {
        // Constant torque and constant thrust (infinite fuel): the exact velocity is a Fresnel-type integral; compare with
        // a fine reference (same propagation with 100x smaller heading steps) instead. The thrust spirals, so the errors are
        // measured against the total impulse a T = 12 m/s^2 * 12 s = 144 m/s, not against the small resultant.
        auto run = [](double headingLimit) {
            FreeSpace F;
            F.env.maxHeadingChangePerStep = headingLimit;
            F.env.activeStepFraction = 1.0;
            ConstantController c(1.0, 1.0);
            F.controllers.set(ControllerKind::Prograde, &c);
            Shuttles s;
            s.infiniteFuel = true;
            REQUIRE(s.add(shuttleDesc(5, {}, {}, ControllerKind::Prograde)));
            propagateShuttle(s, 0, 0.0, 12.0, F.env, F.events);
            return shuttleVel(s, 0);
        };
        const Vec3 ref = run(0.001);
        const Vec3 normal = run(0.1);
        const Vec3 coarse = run(1.0);
        CHECK(norm(normal - ref) < 1e-4 * 144.0); // measured 5.7e-3 m/s = 4e-5 of the impulse
        CHECK(norm(coarse - ref) > 5.0 * norm(normal - ref)); // the limit does work
    }
}
