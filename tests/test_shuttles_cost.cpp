#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include "core/atmosphere.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// What the shuttle logic costs and where it puts its step boundaries: the steps per simulated second of the burn
// autopilots (plan 6.4, 17), the fuel-out boundary (5.7), the step cap in front of an atmosphere (5.6).

namespace {

struct Domus {
    Bodies b;
    LinearEphemeris eph;
    ControllerSet cs;
    ShuttleEnvironment env;
    explicit Domus(bool atmosphere = true) : b(make(atmosphere)), eph(b), env(b, eph, cs) {}
    static Bodies make(bool atmosphere)
    {
        Bodies bodies;
        BodyDesc d = bodyDesc(kMuDomus, kRDomus);
        if (atmosphere) d.atmosphere = Atmosphere{1.22, 7000.0};
        bodies.add(d);
        return bodies;
    }
};

ShuttleObservation observation(double fuel, double spin, bool infinite)
{
    ShuttleObservation o;
    o.shuttleId = 1;
    o.tick = 40;
    o.heading = 0.3;
    o.spin = spin;
    o.fuelMass = fuel;
    o.infiniteFuel = infinite;
    o.maxTorque = 5000.0;
    o.inertia = 1.0e4;
    o.relVelocity = {0.0, 7000.0, 0.0};
    return o;
}

} // namespace

TEST_SUITE("shuttle cost")
{
    TEST_CASE("BurnController with an empty tank damps the spin, then asks for no more ticks")
    {
        ProgradeController c;
        Rng rng = Rng::forTick(1, 40);
        // Fuel on board, or infinite fuel: a state-dependent controller, every tick
        CHECK(c.nextTick(observation(100.0, 0.0, false)) == 41);
        CHECK(c.nextTick(observation(0.0, 0.0, true)) == 41);
        // Tank empty and spinning: the wheels stop the spin (the command opposes it), still called every tick
        const ShuttleObservation spinning = observation(0.0, 0.2, false);
        const ShuttleCommand damp = c.update(spinning, rng);
        CHECK(damp.turn < 0.0);
        CHECK(damp.throttle == 1.0);   // it still asks for thrust: the propagator forces 0 and reports FuelEmpty once
        CHECK(c.nextTick(spinning) == 41);
        // Tank empty and at rest: nothing left to do, no command, never called again
        const ShuttleObservation rest = observation(0.0, 0.5 * BurnController::kRestSpin, false);
        const ShuttleCommand none = c.update(rest, rng);
        CHECK(none.turn == 0.0);
        CHECK(c.nextTick(rest) == std::numeric_limits<std::int64_t>::max());
        // Retrograde is the same
        RetrogradeController r;
        CHECK(r.nextTick(rest) == std::numeric_limits<std::int64_t>::max());
        CHECK(r.nextTick(observation(5.0, 0.0, false)) == 41);
    }

    TEST_CASE("Prograde costs 10 steps per sim second while the engine can run, and nothing once the tank is empty")
    {
        Domus W(false);
        const double r0 = 8.0e5, vc = std::sqrt(kMuDomus / r0);
        auto runFor = [&](double, double t0, double t1, Shuttles& s, ShuttleStats& st) {
            std::vector<Event> ev;
            propagateShuttle(s, 0, t0, t1, W.env, ev, nullptr, &st);
            return ev;
        };

        // (1) With fuel for 30 s of full thrust (mdot 20 kg/s), over the burn: one controller call and one step per tick
        Shuttles s;
        ShuttleDesc d = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Prograde);
        d.heading = PI / 2.0;
        d.fuelMass = 600.0;
        REQUIRE(s.add(d));
        ShuttleStats burn;
        runFor(600.0, 0.0, 29.0, s, burn);
        std::printf("COST Prograde burning: %ld steps, %ld controller calls in 29 s\n", burn.acceptedSteps, burn.controllerTicks);
        CHECK(burn.controllerTicks >= 285);   // 290 ticks of 0.1 s
        CHECK(burn.acceptedSteps >= 285);
        CHECK(burn.acceptedSteps <= 330);     // and no more than a few extra from the heading and active-time limits

        // (2) After the burn-out the shuttle coasts: the steps per second fall to those of a Coast shuttle
        ShuttleStats burnOut, coast;
        std::vector<Event> ev = runFor(0.0, 29.0, 60.0, s, burnOut);   // fuel runs out at 30 s
        CHECK(s.fuelMass[0] == 0.0);
        const std::size_t nEmpty = static_cast<std::size_t>(std::count_if(ev.begin(), ev.end(), [](const Event& e) { return e.type == EventType::FuelEmpty; }));
        CHECK(nEmpty == 1);
        // let the spin damp, then count a long stretch
        runFor(0.0, 60.0, 100.0, s, coast);
        CHECK(std::fabs(s.spin[0]) < 1.0e-6);
        ShuttleStats tail;
        runFor(0.0, 100.0, 1100.0, s, tail);
        std::printf("COST Prograde after burn-out: %ld steps, %ld controller calls in 1000 s (10 per second would be 10000)\n",
                    tail.acceptedSteps, tail.controllerTicks);
        CHECK(tail.controllerTicks == 0);
        CHECK(tail.acceptedSteps < 400);   // the gravity step alone: about 150 per orbit of 1.9e3 s at rtol 1e-10

        // The same with a Coast shuttle on the same orbit, for scale
        Shuttles c;
        ShuttleDesc dc = shuttleDesc(2, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Coast);
        REQUIRE(c.add(dc));
        ShuttleStats cs;
        std::vector<Event> ev2;
        propagateShuttle(c, 0, 0.0, 1000.0, W.env, ev2, nullptr, &cs);
        CHECK(tail.acceptedSteps < 3 * cs.acceptedSteps + 20);
    }

    TEST_CASE("with infinite fuel Prograde keeps its tick rate (the price of a state-dependent autopilot)")
    {
        Domus W(false);
        W.env.activeStepFraction = 0.0;
        const double r0 = 8.0e5, vc = std::sqrt(kMuDomus / r0);
        Shuttles s;
        s.infiniteFuel = true;
        ShuttleDesc d = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Prograde);
        d.heading = PI / 2.0;
        REQUIRE(s.add(d));
        ShuttleStats st;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 100.0, W.env, ev, nullptr, &st);
        CHECK(st.controllerTicks >= 995);
        CHECK(st.controllerTicks <= 1001);
    }

    TEST_CASE("the step ends where the fuel runs out: FuelEmpty on a step boundary and the tank exactly empty")
    {
        // Full thrust along a fixed heading from a circular orbit; fuel for 17.3 s (mdot 20 kg/s -> 346 kg)
        Domus W(false);
        const double r0 = 8.0e5, vc = std::sqrt(kMuDomus / r0);
        ConstantController full(1.0, 0.0);
        ControllerSet cs;
        cs.set(ControllerKind::Wander, &full);
        ShuttleEnvironment env(W.b, W.eph, cs);
        env.activeStepFraction = 0.02;
        Shuttles s;
        ShuttleDesc d = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Wander);
        d.heading = PI / 2.0;
        d.fuelMass = 346.0;
        REQUIRE(s.add(d));
        StepRecorder rec;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 60.0, env, ev, &rec);
        REQUIRE(ev.size() == 1);
        REQUIRE(ev[0].type == EventType::FuelEmpty);
        const double tEmpty = 346.0 / 20.0;
        CHECK(ev[0].t == doctest::Approx(tEmpty).epsilon(1e-8));
        // The event time is the end of an accepted step: the burn does not stop inside a step
        double nearest = std::numeric_limits<double>::infinity();
        for (const ShuttleStepInfo& st : rec.steps) nearest = std::min(nearest, std::fabs(st.t1 - ev[0].t));
        std::printf("FUELOUT event at %.9f s, nearest step end %.3e s away, %zu steps\n", ev[0].t, nearest, rec.steps.size());
        CHECK(nearest < 1.0e-6);
        CHECK(s.fuelMass[0] == 0.0);
        // The delta-v is the whole tank
        const double dv = 3000.0 * std::log(2346.0 / 2000.0);
        const Vec3 vEnd = shuttleVel(s, 0);
        // after the burn the shuttle coasts: speed gain bounded by dv from both sides (gravity turns the velocity)
        CHECK(norm(vEnd) > vc);
        CHECK(norm(vEnd) < vc + dv + 1.0);
    }

    TEST_CASE("circular orbit just above an atmosphere: steps at the natural rate, not at the dive rate")
    {
        // The step cap in front of the layer used to assume a straight flight at it. For a circular orbit at 1 km above the
        // top it cut the steps to the inside limit (2710 per 1000 s measured); the radial closing speed is zero here.
        auto stepsFor = [](double above) {
            Domus W(true);
            const double top = atmosphereTop(*W.b.atmosphere[0]);
            const double r0 = kRDomus + top + above, vc = std::sqrt(kMuDomus / r0);
            Shuttles s;
            REQUIRE(s.add(shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0})));
            ShuttleStats st;
            std::vector<Event> ev;
            propagateShuttle(s, 0, 0.0, 1000.0, W.env, ev, nullptr, &st);
            return st.acceptedSteps;
        };
        const long atKm = stepsFor(1.0e3), at20 = stepsFor(2.0e4), at300 = stepsFor(3.0e5);
        std::printf("ATMCAP steps per 1000 s at 1 / 20 / 300 km above the top: %ld / %ld / %ld\n", atKm, at20, at300);
        CHECK(atKm < 200);
        CHECK(at20 < 200);
        CHECK(atKm < 3 * at300 + 50);
    }

    TEST_CASE("a fall into the atmosphere from rest above it still cannot jump the layer in one step")
    {
        // The cap uses the pull at the layer top (the strongest on the way), so it cannot overshoot a radial fall: from rest at
        // 300 km above the top the shuttle reaches the layer at about 6.7 km/s and a step must end before it is deep inside.
        Domus W(true);
        const double top = atmosphereTop(*W.b.atmosphere[0]);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {kRDomus + top + 3.0e5, 0.0, 0.0}, {0.0, 0.0, 0.0})));
        StepRecorder rec;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 400.0, W.env, ev, &rec);
        std::size_t k = 0;
        while (k < rec.steps.size() && norm(rec.steps[k].r1) > kRDomus + top) ++k;
        REQUIRE(k < rec.steps.size());
        const double depth = (kRDomus + top) - norm(rec.steps[k].r1);
        std::printf("ATMCAP fall from rest: first step into the air ends %.1f m deep (layer %.0f m)\n", depth, top);
        CHECK(depth < 0.05 * top);
    }
}
