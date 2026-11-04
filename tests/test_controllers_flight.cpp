#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Autopilots flying through propagateShuttle (plan 7.3): pointing settles, Prograde raises and Retrograde lowers the
// orbit around a static planet, Wander is reproducible.

namespace {
struct Settle {
    double settleTime = -1.0;   // first time after which |error| < 0.01 rad and |spin| < 0.01 rad/s for good
    double maxOvershoot = 0.0;  // largest error on the far side of the target, rad
    double finalError = 0.0;
    double finalSpin = 0.0;
};

// Free space, no thrust: a Prograde shuttle moving along +x (target heading 0) starting with the given heading error.
Settle pointFrom(double initialHeading, double inertia, double maxTorque)
{
    Bodies none;
    LinearEphemeris eph(none);
    ControllerSet cs;
    ShuttleEnvironment env(none, eph, cs);
    Shuttles s;
    ShuttleDesc d = shuttleDesc(1, {0.0, 0.0, 0.0}, {1000.0, 0.0, 0.0}, ControllerKind::Prograde);
    d.heading = initialHeading;
    d.maxThrust = 0.0;
    d.inertia = inertia;
    d.maxTorque = maxTorque;
    REQUIRE(s.add(d));
    std::vector<Event> ev;
    Settle out;
    double lastBad = 0.0;
    const double T = 40.0;
    const double dt = 0.1;
    const int N = static_cast<int>(T / dt + 0.5);
    const double sign0 = initialHeading > 0.0 ? 1.0 : -1.0;
    for (int k = 0; k < N; ++k) {
        propagateShuttle(s, 0, k * dt, (k + 1) * dt, env, ev);
        const double err = wrapAngle(0.0 - s.heading[0]);
        const double t = (k + 1) * dt;
        if (std::fabs(err) >= 0.01 || std::fabs(s.spin[0]) >= 0.01) lastBad = t;
        out.maxOvershoot = std::max(out.maxOvershoot, -sign0 * s.heading[0]); // heading past the target (0)
    }
    out.settleTime = lastBad;
    out.finalError = std::fabs(wrapAngle(0.0 - s.heading[0]));
    out.finalSpin = std::fabs(s.spin[0]);
    return out;
}

// Specific orbital energy about a body at the origin
double energy(const Shuttles& s, double mu)
{
    return 0.5 * norm2(shuttleVel(s, 0)) - mu / norm(shuttlePos(s, 0));
}

struct BurnRun {
    double dEnergy = 0.0;
    double fuelUsed = 0.0;
    double sma0 = 0.0, sma1 = 0.0;
};

BurnRun burnAroundPlanet(ControllerKind kind, double duration, double r0)
{
    Bodies b = singleBody(kMuDomus, kRDomus);
    LinearEphemeris eph(b);
    ControllerSet cs;
    ShuttleEnvironment env(b, eph, cs);
    Shuttles s;
    const double vc = std::sqrt(kMuDomus / r0);
    ShuttleDesc d = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, kind);
    d.heading = PI / 2.0; // prograde at the start
    REQUIRE(s.add(d));
    BurnRun out;
    const double e0 = energy(s, kMuDomus);
    const double f0 = s.fuelMass[0];
    std::vector<Event> ev;
    propagateShuttle(s, 0, 0.0, duration, env, ev);
    out.dEnergy = energy(s, kMuDomus) - e0;
    out.fuelUsed = f0 - s.fuelMass[0];
    out.sma0 = -kMuDomus / (2.0 * e0);
    out.sma1 = -kMuDomus / (2.0 * energy(s, kMuDomus));
    return out;
}
} // namespace

TEST_SUITE("controllers in flight")
{
    TEST_CASE("PD pointing settles from errors up to 180 degrees with the 7.1 defaults")
    {
        // Measured (10 Hz sample-and-hold, alpha = 0.5 rad/s^2): settled below 0.01 rad and 0.01 rad/s after 4.1 s (0.2 rad
        // error) to 7.4 s (3.1 rad), overshoot at most 1.9e-3 rad, rest to 1e-15 (zero command at zero error and zero spin).
        for (double h0 : {0.2, -0.2, 1.0, -1.0, 2.0, -2.5, 3.1, -3.1}) {
            const Settle r = pointFrom(h0, 1.0e4, 5000.0);
            CAPTURE(h0);
            CHECK(r.settleTime <= 9.0);
            CHECK(r.finalError < 1e-9);
            CHECK(r.finalSpin < 1e-9);
            CHECK(r.maxOvershoot < 0.01);
        }
    }

    TEST_CASE("PD pointing also settles for a heavier and a lighter wheel set (gains rescale with the authority)")
    {
        for (double inertia : {4.0e4, 2.5e3}) {
            const Settle r = pointFrom(2.0, inertia, 5000.0);
            CAPTURE(inertia);
            CHECK(r.finalError < 1e-2);
            CHECK(r.finalSpin < 1e-2);
        }
    }

    TEST_CASE("Prograde burns tangentially and raises the orbit; the energy gain is the thrust work on the orbital speed")
    {
        // 20 s at full thrust from a circular orbit at 800 km radius: 400 kg of fuel, rocket dv = 3000 ln(5000/4600) = 250 m/s
        const double r0 = 8.0e5;
        const BurnRun run = burnAroundPlanet(ControllerKind::Prograde, 20.0, r0);
        const double vc = std::sqrt(kMuDomus / r0);
        const double dv = 3000.0 * std::log(5000.0 / 4600.0);
        CHECK(run.fuelUsed == doctest::Approx(400.0).epsilon(1e-6)); // starts aligned: burns the whole time
        CHECK(run.dEnergy > 0.0);
        CHECK(run.sma1 > run.sma0);
        // Thrust along the velocity does work a_T |v| dt: between dv * vc and dv * (vc + dv)
        CHECK(run.dEnergy > dv * vc);
        CHECK(run.dEnergy < dv * (vc + dv));
        // Measured: sma 8.0e5 -> 1.058e6 (the impulsive prediction from vis-viva is 1.058e6 too)
        CHECK(run.sma1 > 1.0e6);
        CHECK(run.sma1 < 1.12e6);
    }

    TEST_CASE("Retrograde turns around first, then lowers the orbit")
    {
        const double r0 = 8.0e5;
        const BurnRun run = burnAroundPlanet(ControllerKind::Retrograde, 40.0, r0);
        CHECK(run.dEnergy < 0.0);
        CHECK(run.sma1 < run.sma0);
        // It started facing the wrong way, so it burned for less than the whole 40 s (measured: 712 kg of the 800 kg, so the
        // 180 degree turn took 4.4 s)
        CHECK(run.fuelUsed < 800.0 - 50.0);
        CHECK(run.fuelUsed > 400.0); // but burned for well over 20 s
    }

    TEST_CASE("Retrograde from an aligned start is the mirror image of Prograde, apart from the work done at a different speed")
    {
        Bodies b = singleBody(kMuDomus, kRDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        const double r0 = 3.0e6, vc = std::sqrt(kMuDomus / r0);
        Shuttles s;
        ShuttleDesc up = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Prograde);
        up.heading = PI / 2.0;
        ShuttleDesc down = shuttleDesc(2, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Retrograde);
        down.heading = -PI / 2.0;
        REQUIRE(s.add(up));
        REQUIRE(s.add(down));
        std::vector<Event> ev;
        const double e0 = 0.5 * vc * vc - kMuDomus / r0;
        for (std::size_t i = 0; i < 2; ++i) propagateShuttle(s, i, 0.0, 15.0, env, ev);
        auto en = [&](std::size_t i) { return 0.5 * norm2(shuttleVel(s, i)) - kMuDomus / norm(shuttlePos(s, i)); };
        const double gain = en(0) - e0, loss = e0 - en(1);
        CHECK(gain > 0.0);
        CHECK(loss > 0.0);
        // Thrust along the velocity does work a_T |v| dt, so with the same dv the prograde burn gains
        // (vc + dv/2) / (vc - dv/2) times what the retrograde burn loses (measured 1.181, predicted 1.181)
        const double dv = 3000.0 * std::log(5000.0 / 4700.0);
        const double predicted = (vc + 0.5 * dv) / (vc - 0.5 * dv);
        CHECK(gain / loss == doctest::Approx(predicted).epsilon(0.02));
        CHECK(s.fuelMass[0] == doctest::Approx(s.fuelMass[1]).epsilon(1e-6));
    }

    TEST_CASE("Wander: fuel burned equals the segments' commands (hold semantics), and runs are reproducible")
    {
        Bodies b = singleBody(kMuDomus, kRDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        const double r0 = 3.0e6, vc = std::sqrt(kMuDomus / r0);
        const double T = 200.0; // 100 segments of 2 s

        auto make = [&](ShuttleId id) {
            Shuttles s;
            ShuttleDesc d = shuttleDesc(id, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Wander);
            REQUIRE(s.add(d));
            return s;
        };
        // Whole interval in one call
        Shuttles whole = make(31);
        std::vector<Event> ev;
        propagateShuttle(whole, 0, 0.0, T, env, ev);

        // The same, in 11 uneven pieces
        Shuttles pieces = make(31);
        const double cuts[] = {0.0, 0.05, 3.33, 19.9, 20.0, 57.123, 80.0, 123.4, 150.0, 199.95, 200.0};
        for (int k = 0; k < 10; ++k) propagateShuttle(pieces, 0, cuts[k], cuts[k + 1], env, ev);

        // Expected fuel: sum over the 100 segments of throttle * 2 s * mdot (mdot = 60000 / 3000 = 20 kg/s at full throttle)
        WanderController w;
        double expected = 0.0;
        for (int seg = 0; seg < 100; ++seg) expected += w.commandFor(31, seg * 20).throttle * 2.0 * 20.0;
        REQUIRE(expected > 100.0);
        REQUIRE(expected < 2500.0); // does not run dry
        CHECK(3000.0 - whole.fuelMass[0] == doctest::Approx(expected).epsilon(1e-9));
        CHECK(3000.0 - pieces.fuelMass[0] == doctest::Approx(expected).epsilon(1e-9));

        // Same commands, different step boundaries: the same flight to integration accuracy
        // (measured 5e-3 m and 7e-5 m/s after 200 s with ~1000 m/s of thrust: the splitting error of a 2 s burn step, h^3 a_T mu / r^3)
        CHECK(norm(shuttlePos(whole, 0) - shuttlePos(pieces, 0)) < 5e-2);
        CHECK(norm(shuttleVel(whole, 0) - shuttleVel(pieces, 0)) < 5e-4);
        CHECK(whole.heading[0] == doctest::Approx(pieces.heading[0]).epsilon(1e-9));

        // Rerunning the identical schedule is bit-identical
        Shuttles again = make(31);
        for (int k = 0; k < 10; ++k) propagateShuttle(again, 0, cuts[k], cuts[k + 1], env, ev);
        CHECK(sameBits(again.state.x[0], pieces.state.x[0]));
        CHECK(sameBits(again.state.vy[0], pieces.state.vy[0]));
        CHECK(sameBits(again.heading[0], pieces.heading[0]));
        CHECK(sameBits(again.fuelMass[0], pieces.fuelMass[0]));

        // Another shuttle id wanders differently
        Shuttles other = make(32);
        propagateShuttle(other, 0, 0.0, T, env, ev);
        CHECK(norm(shuttleVel(other, 0) - shuttleVel(whole, 0)) > 1.0);
    }

    TEST_CASE("Coast leaves the orbit alone, and does not make the steps stop at controller ticks")
    {
        Bodies b = singleBody(kMuDomus, kRDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        const double r0 = 8.0e5, vc = std::sqrt(kMuDomus / r0);
        auto run = [&](double tick, long& steps) {
            ShuttleEnvironment env(b, eph, cs);
            env.controllerTick = tick;
            Shuttles s;
            REQUIRE(s.add(shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, vc, 0.0}, ControllerKind::Coast)));
            std::vector<Event> ev;
            ShuttleStats st;
            propagateShuttle(s, 0, 0.0, 10000.0, env, ev, nullptr, &st);
            steps = st.acceptedSteps;
            return s;
        };
        long stepsFast = 0, stepsSlow = 0;
        const Shuttles a = run(0.1, stepsFast);
        const Shuttles c = run(1e6, stepsSlow);
        CHECK(stepsFast == stepsSlow);               // the 0.1 s ticks are not step boundaries
        CHECK(stepsFast < 800);                     // 143 per orbit (2316 s), 4.3 orbits: about 620
        CHECK(sameBits(a.state.x[0], c.state.x[0])); // not even a rounding difference
        CHECK(sameBits(a.state.vy[0], c.state.vy[0]));
        CHECK(a.heading[0] == 0.0);
        CHECK(a.fuelMass[0] == 3000.0);
    }

    TEST_CASE("burn-gravity splitting is second order in the step; the default active step keeps it to centimetres per second")
    {
        // 100 s of full thrust along a fixed inertial heading from a circular orbit at 800 km radius (dynamical time 369 s, the
        // gravity step alone would take ~16 s steps). The step while thrusting is activeStepFraction * 369 s. Reference: 0.25e-3
        // of it (0.09 s). Measured position / velocity differences to the reference:
        //   fraction 0.02: 20 m / 4.4e-2 m/s    0.01: 5.1 m / 1.1e-2    0.005: 1.3 m / 2.8e-3    0.002: 0.20 m / 4.5e-4
        auto fly = [](double fraction) {
            Bodies b = singleBody(kMuDomus, kRDomus);
            LinearEphemeris eph(b);
            ControllerSet cs;
            ConstantController full(1.0, 0.0);
            cs.set(ControllerKind::Prograde, &full);
            ShuttleEnvironment env(b, eph, cs);
            env.activeStepFraction = fraction;
            Shuttles s;
            const double r0 = 8.0e5;
            ShuttleDesc d = shuttleDesc(1, {r0, 0.0, 0.0}, {0.0, std::sqrt(kMuDomus / r0), 0.0}, ControllerKind::Prograde);
            d.heading = PI / 2.0;
            REQUIRE(s.add(d));
            std::vector<Event> ev;
            propagateShuttle(s, 0, 0.0, 100.0, env, ev);
            return s;
        };
        const Shuttles ref = fly(0.00025);
        auto posErr = [&](double f) { return norm(shuttlePos(fly(f), 0) - shuttlePos(ref, 0)); };
        auto velErr = [&](double f) { return norm(shuttleVel(fly(f), 0) - shuttleVel(ref, 0)); };
        double prevP = posErr(0.02), prevV = velErr(0.02);
        for (double f : {0.01, 0.005, 0.002}) {
            const double p = posErr(f), v = velErr(f);
            CAPTURE(f);
            if (f > 0.004) { // halving: ratio 4 for a second-order scheme (measured 3.9 .. 4.0)
                CHECK(prevP / p > 3.6);
                CHECK(prevP / p < 4.4);
                CHECK(prevV / v > 3.6);
                CHECK(prevV / v < 4.4);
            }
            prevP = p;
            prevV = v;
        }
        // The default (activeStepFraction = 0.005): 1.3 m and 3 mm/s after 100 s of burn
        const Bodies none;
        const LinearEphemeris eph(none);
        const ControllerSet cs;
        const ShuttleEnvironment defaults(none, eph, cs);
        CHECK(defaults.activeStepFraction == 0.005);
        CHECK(posErr(defaults.activeStepFraction) < 2.0);
        CHECK(velErr(defaults.activeStepFraction) < 5e-3);
    }
}
