#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "core/bodies.h"
#include "core/gravity.h"
#include "core/ias15.h"

using namespace cosmos;

// Stage 1 of IAS15 (plan section 6.3): stepIAS15Fixed, no step-size control.
// Tolerances below are set from measured values (release and debug builds agree to the last digit shown).

namespace {
const double TWO_PI = 6.283185307179586476925286766559;

BodyDesc body(double mu, double x, double y, double vx, double vy)
{
    BodyDesc d;
    d.mu = mu;
    d.position = {x, y, 0.0};
    d.velocity = {vx, vy, 0.0};
    return d;
}

// mu = 1 at the origin (held fixed because the orbiter is massless) and an orbiter at periapsis, semi-major axis 1.
Bodies keplerPair(double e)
{
    Bodies b;
    b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
    b.add(body(0.0, 1.0 - e, 0.0, 0.0, std::sqrt((1.0 + e) / (1.0 - e))));
    return b;
}

double orbiterEnergy(const BodyState& s)
{
    const double r = std::hypot(s.x[1], s.y[1]);
    return 0.5 * (s.vx[1] * s.vx[1] + s.vy[1] * s.vy[1]) - 1.0 / r;
}

double orbiterL(const BodyState& s) { return s.x[1] * s.vy[1] - s.y[1] * s.vx[1]; }

// Distance of the orbiter from its start after `steps` fixed steps of one orbit / stepsPerOrbit each
double returnError(double e, int stepsPerOrbit, int steps)
{
    Bodies b = keplerPair(e);
    const BodyState s0 = b.state;
    Accel a;
    computeAccelerations(b.state, b.mu, a);
    for (int k = 0; k < steps; ++k) stepIAS15Fixed(b.state, b.mu, a, TWO_PI / stepsPerOrbit);
    return std::hypot(b.state.x[1] - s0.x[1], b.state.y[1] - s0.y[1]);
}
}

TEST_SUITE("ias15 fixed step")
{
    TEST_CASE("circular orbit, 8 steps per orbit: back at the start to 5e-14")
    {
        // measured 2.2e-15; a 15th-order method needs only 8 steps per orbit to reach roundoff here
        CHECK(returnError(0.0, 8, 8) < 5e-14);
    }

    TEST_CASE("15th order: halving dt (2 vs 4 steps per orbit) cuts the error by about 2^15")
    {
        const double e2 = returnError(0.0, 2, 2);
        const double e4 = returnError(0.0, 4, 4);
        CHECK(e2 > 1e-10);            // measured 2.0e-9: the coarse run really is truncation-limited
        const double ratio = e2 / e4; // measured 2.5e4 (2^15 = 32768); order 13 to 17 would give 8192 to 131072
        CHECK(ratio > 8192.0);
        CHECK(ratio < 131072.0);
    }

    TEST_CASE("eccentric orbit e = 0.5, 32 steps per orbit: one-orbit return error below 1e-12")
    {
        // measured 3.2e-13
        CHECK(returnError(0.5, 32, 32) < 1e-12);
    }

    TEST_CASE("e = 0.5, 64 steps per orbit, 200 orbits: energy and angular momentum stay at roundoff")
    {
        Bodies b = keplerPair(0.5);
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        const double E0 = orbiterEnergy(b.state), L0 = orbiterL(b.state);
        double dE = 0.0, dL = 0.0;
        for (int k = 0; k < 64 * 200; ++k) {
            stepIAS15Fixed(b.state, b.mu, a, TWO_PI / 64);
            dE = std::max(dE, std::abs((orbiterEnergy(b.state) - E0) / E0));
            dL = std::max(dL, std::abs((orbiterL(b.state) - L0) / L0));
        }
        CHECK(dE < 1e-13); // measured 2.4e-14
        CHECK(dL < 5e-14); // measured 9e-15
    }

    TEST_CASE("time reversible: 40 steps forward then 40 back returns to the start")
    {
        Bodies b = keplerPair(0.5);
        const BodyState s0 = b.state;
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        const double dt = TWO_PI / 64;
        for (int k = 0; k < 40; ++k) stepIAS15Fixed(b.state, b.mu, a, dt);
        CHECK(std::hypot(b.state.x[1] - s0.x[1], b.state.y[1] - s0.y[1]) > 0.5); // it really moved away
        for (int k = 0; k < 40; ++k) stepIAS15Fixed(b.state, b.mu, a, -dt);
        CHECK(std::hypot(b.state.x[1] - s0.x[1], b.state.y[1] - s0.y[1]) < 3e-14);
        CHECK(std::hypot(b.state.vx[1] - s0.vx[1], b.state.vy[1] - s0.vy[1]) < 3e-14);
    }

    TEST_CASE("a is valid on return: equal to computeAccelerations at the new positions")
    {
        Bodies b = keplerPair(0.5);
        Accel a, ref;
        computeAccelerations(b.state, b.mu, a);
        for (int k = 0; k < 5; ++k) {
            stepIAS15Fixed(b.state, b.mu, a, 0.3);
            computeAccelerations(b.state, b.mu, ref);
            for (std::size_t i = 0; i < 2; ++i) {
                CHECK(a.ax[i] == ref.ax[i]);
                CHECK(a.ay[i] == ref.ay[i]);
                CHECK(a.az[i] == ref.az[i]);
            }
        }
    }

    TEST_CASE("returns the number of rounds: at least 1, at most the cap of 25, fewer for an easy step")
    {
        Bodies b = keplerPair(0.0);
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        const int hard = stepIAS15Fixed(b.state, b.mu, a, TWO_PI / 4); // large step: needs many rounds
        CHECK(hard >= 1);
        CHECK(hard <= 25);

        Bodies c = keplerPair(0.0);
        Accel ac;
        computeAccelerations(c.state, c.mu, ac);
        const int easy = stepIAS15Fixed(c.state, c.mu, ac, TWO_PI / 4096); // tiny step: converges in a few rounds
        CHECK(easy >= 2);
        CHECK(easy <= 8); // measured 4 (the large step above measured 14)
        CHECK(easy < hard);
    }

    TEST_CASE("three bodies: total momentum is conserved to roundoff")
    {
        Bodies b;
        b.add(body(1.0, 0.0, 0.0, 0.0, -0.1));
        b.add(body(0.3, 1.0, 0.2, 0.0, 1.0));
        b.add(body(0.2, -1.5, 0.1, 0.1, -0.7));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        auto momentum = [&](double& px, double& py) {
            px = py = 0.0;
            for (std::size_t i = 0; i < 3; ++i) {
                px += b.mu[i] * b.state.vx[i];
                py += b.mu[i] * b.state.vy[i];
            }
        };
        double px0, py0, px, py;
        momentum(px0, py0);
        for (int k = 0; k < 200; ++k) stepIAS15Fixed(b.state, b.mu, a, 0.02);
        momentum(px, py);
        CHECK(std::hypot(px - px0, py - py0) < 1e-14);
    }

    TEST_CASE("z stays exactly 0 in a planar problem")
    {
        Bodies b = keplerPair(0.5);
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        for (int k = 0; k < 50; ++k) stepIAS15Fixed(b.state, b.mu, a, 0.1);
        for (std::size_t i = 0; i < 2; ++i) {
            CHECK(b.state.z[i] == 0.0);
            CHECK(b.state.vz[i] == 0.0);
        }
    }

    TEST_CASE("max|a| = 0 (all mu zero): straight-line drift, 0 rounds, no NaN")
    {
        Bodies b;
        b.add(body(0.0, 0.0, 0.0, 1.0, 2.0));
        b.add(body(0.0, 5.0, 5.0, -1.0, 0.5));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        const int rounds = stepIAS15Fixed(b.state, b.mu, a, 0.25);
        CHECK(rounds == 0);
        CHECK(b.state.x[0] == 0.25);
        CHECK(b.state.y[0] == 0.5);
        CHECK(b.state.x[1] == 4.75);
        CHECK(b.state.y[1] == 5.125);
        CHECK(b.state.vx[1] == -1.0);
        CHECK(a.ax[0] == 0.0);
    }
}
