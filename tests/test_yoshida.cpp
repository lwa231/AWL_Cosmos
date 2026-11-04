#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "core/bodies.h"
#include "core/diagnostics.h"
#include "core/gravity.h"
#include "core/integrators.h"

using namespace cosmos;

namespace {
const double TWO_PI = 6.283185307179586476925286766559;

using Stepper = void (*)(BodyState&, const std::vector<double>&, Accel&, double);

BodyDesc body(double mu, double x, double y, double vx, double vy)
{
    BodyDesc d;
    d.mu = mu;
    d.position = {x, y, 0.0};
    d.velocity = {vx, vy, 0.0};
    return d;
}

// Test particle (mu = 0) on a circular orbit of radius 1 around mu = 1; distance from the start after one period.
double circularErrorAfterOnePeriod(Stepper step, int steps)
{
    Bodies b;
    b.add(body(1.0, 0, 0, 0, 0));
    b.add(body(0.0, 1, 0, 0, 1));
    Accel a;
    computeAccelerations(b.state, b.mu, a);
    const double dt = TWO_PI / steps;
    for (int k = 0; k < steps; ++k) step(b.state, b.mu, a, dt);
    return std::hypot(b.state.x[1] - 1.0, b.state.y[1]);
}

// Same but e = 0.5 started at periapsis (r = 0.5, v = sqrt(3)); distance from the start after one period.
double eccentricErrorAfterOnePeriod(Stepper step, int steps)
{
    Bodies b;
    b.add(body(1.0, 0, 0, 0, 0));
    b.add(body(0.0, 0.5, 0, 0, std::sqrt(3.0)));
    Accel a;
    computeAccelerations(b.state, b.mu, a);
    const double dt = TWO_PI / steps;
    for (int k = 0; k < steps; ++k) step(b.state, b.mu, a, dt);
    return std::hypot(b.state.x[1] - 0.5, b.state.y[1]);
}
}

TEST_SUITE("yoshida4")
{
    TEST_CASE("sub-step weights add up to dt: a free particle moves exactly v*dt")
    {
        Bodies b;
        b.add(body(0.0, 1.0, 2.0, 3.0, -2.0));   // mu = 0 and alone: no acceleration at all
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        stepYoshida4(b.state, b.mu, a, 0.25);
        CHECK(std::abs(b.state.x[0] - (1.0 + 3.0 * 0.25)) < 1e-14);
        CHECK(std::abs(b.state.y[0] - (2.0 - 2.0 * 0.25)) < 1e-14);
        CHECK(std::abs(b.state.vx[0] - 3.0) < 1e-14);
        CHECK(std::abs(b.state.vy[0] + 2.0) < 1e-14);
    }

    TEST_CASE("circular orbit returns to its start after one period")
    {
        CHECK(circularErrorAfterOnePeriod(stepYoshida4, 1000) < 1e-7);
    }

    TEST_CASE("fourth order: halving dt cuts the circular-orbit error by about 16")
    {
        const double ratio = circularErrorAfterOnePeriod(stepYoshida4, 1000)
                           / circularErrorAfterOnePeriod(stepYoshida4, 2000);
        CHECK(ratio > 15.0);
        CHECK(ratio < 17.0);
    }

    TEST_CASE("fourth order: halving dt cuts the e = 0.5 error by about 16")
    {
        const double ratio = eccentricErrorAfterOnePeriod(stepYoshida4, 2000)
                           / eccentricErrorAfterOnePeriod(stepYoshida4, 4000);
        std::printf("[validation] Yoshida e=0.5 order: error ratio 2000/4000 steps %.4f (errors %.4g, %.4g)\n", ratio,
                    eccentricErrorAfterOnePeriod(stepYoshida4, 2000), eccentricErrorAfterOnePeriod(stepYoshida4, 4000));
        CHECK(ratio > 15.0);
        CHECK(ratio < 17.0);
    }

    TEST_CASE("far more accurate than KDK at the same dt")
    {
        CHECK(circularErrorAfterOnePeriod(stepYoshida4, 1000) * 1000.0
              < circularErrorAfterOnePeriod(stepKDK, 1000));
    }

    TEST_CASE("eccentric orbit e = 0.5: returns to periapsis, energy stays bounded")
    {
        Bodies b;
        b.add(body(1.0, 0, 0, 0, 0));
        b.add(body(0.0, 0.5, 0, 0, std::sqrt(3.0)));
        Accel a;
        computeAccelerations(b.state, b.mu, a);

        const int steps = 2000;
        const double dt = TWO_PI / steps;
        double emin = 1e300, emax = -1e300;
        for (int k = 0; k < steps; ++k) {
            stepYoshida4(b.state, b.mu, a, dt);
            const double v2 = b.state.vx[1] * b.state.vx[1] + b.state.vy[1] * b.state.vy[1];
            const double e = 0.5 * v2 - 1.0 / std::hypot(b.state.x[1], b.state.y[1]);
            emin = std::min(emin, e);
            emax = std::max(emax, e);
        }
        CHECK(std::hypot(b.state.x[1] - 0.5, b.state.y[1]) < 1e-6);
        CHECK(emax - emin < 1e-7);
        CHECK(emin == doctest::Approx(-0.5).epsilon(1e-6).scale(0));
    }

    TEST_CASE("equal-mass circular binary conserves E, P and L")
    {
        Bodies b;
        b.add(body(0.5, -0.5, 0, 0, -0.5));
        b.add(body(0.5, 0.5, 0, 0, 0.5));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        const Conserved c0 = conserved(b);

        const int steps = 1000;
        const double dt = TWO_PI / steps;
        for (int k = 0; k < steps; ++k) stepYoshida4(b.state, b.mu, a, dt);
        const Conserved c1 = conserved(b);

        CHECK(std::abs(c1.energyG - c0.energyG) <= 1e-12 * std::abs(c0.energyG));
        CHECK(std::abs(c1.angularMomentumG.z - c0.angularMomentumG.z) <= 1e-12 * std::abs(c0.angularMomentumG.z));
        CHECK(std::abs(c1.momentumG.x) < 1e-12);
        CHECK(std::abs(c1.momentumG.y) < 1e-12);
        CHECK(std::hypot(b.state.x[1] - b.state.x[0], b.state.y[1] - b.state.y[0]) == doctest::Approx(1.0).epsilon(1e-8).scale(0));
    }

    TEST_CASE("time reversible: stepping back with -dt recovers the start")
    {
        Bodies b;
        b.add(body(1.0, 0, 0, 0, 0));
        b.add(body(0.0, 0.5, 0, 0, std::sqrt(3.0)));
        Accel a;
        computeAccelerations(b.state, b.mu, a);

        const int steps = 500;
        const double dt = TWO_PI / 2000;
        for (int k = 0; k < steps; ++k) stepYoshida4(b.state, b.mu, a, dt);
        for (int k = 0; k < steps; ++k) stepYoshida4(b.state, b.mu, a, -dt);

        CHECK(std::hypot(b.state.x[1] - 0.5, b.state.y[1]) < 1e-10);
        CHECK(std::hypot(b.state.vx[1], b.state.vy[1] - std::sqrt(3.0)) < 1e-10);
    }
}
