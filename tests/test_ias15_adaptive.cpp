#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "core/bodies.h"
#include "core/constants.h"
#include "core/gravity.h"
#include "core/ias15.h"
#include "core/kepler.h"

using namespace cosmos;

// Stages 2 and 3 of IAS15 (plan sections 6.3 and 14): the adaptive integrator class, its validation problems and its
// dense output. Tolerances are set from measured values (the comment next to each says what was measured).

namespace {
const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;

BodyDesc body(double mu, double x, double y, double vx, double vy)
{
    BodyDesc d;
    d.mu = mu;
    d.position = {x, y, 0.0};
    d.velocity = {vx, vy, 0.0};
    return d;
}

ForceFn gravity(const Bodies& b)
{
    return [&b](const BodyState& s, Accel& a) { computeAccelerations(s, b.mu, a); };
}

// G * (kinetic + potential energy), the same form as conserved() in diagnostics.h, for any state and mu list
double energy(const BodyState& s, const std::vector<double>& mu)
{
    double e = 0.0;
    const std::size_t n = s.x.size();
    for (std::size_t i = 0; i < n; ++i) e += 0.5 * mu[i] * ((s.vx[i] * s.vx[i] + s.vy[i] * s.vy[i]) + s.vz[i] * s.vz[i]);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = s.x[j] - s.x[i], dy = s.y[j] - s.y[i], dz = s.z[j] - s.z[i];
            e -= mu[i] * mu[j] / std::sqrt((dx * dx + dy * dy) + dz * dz);
        }
    return e;
}

double angularMomentum(const BodyState& s, const std::vector<double>& mu)
{
    double l = 0.0;
    for (std::size_t i = 0; i < s.x.size(); ++i) l += mu[i] * (s.x[i] * s.vy[i] - s.y[i] * s.vx[i]);
    return l;
}

// Central body (mu = 1, fixed because the orbiter is massless) and an orbiter at periapsis, a = 1, mu = 1
Bodies keplerPair(double e)
{
    Bodies b;
    b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
    b.add(body(0.0, 1.0 - e, 0.0, 0.0, std::sqrt((1.0 + e) / (1.0 - e))));
    return b;
}

// Specific energy of the massless orbiter (body 1) in the field of the central body (mu = 1). energy() would be 0 here.
double orbiterEnergy(const BodyState& s)
{
    return 0.5 * (s.vx[1] * s.vx[1] + s.vy[1] * s.vy[1]) - 1.0 / std::hypot(s.x[1], s.y[1]);
}

struct KeplerRun {
    double maxEnergyError = 0.0;   // max |dE / E| over the run (sampled each step)
    double periodError = 0.0;      // |mean-anomaly shift after the whole run| / (2 pi * orbits): relative period error
    double returnError = 0.0;      // distance from the start after exactly `orbits` periods
    double finalTimeError = 0.0;
    long steps = 0;
};

KeplerRun runKepler(double e, int orbits, double epsilon = 1e-9)
{
    Bodies b = keplerPair(e);
    const BodyState s0 = b.state;
    IAS15Params p;
    p.epsilon = epsilon;
    IAS15 ias(p);
    const ForceFn f = gravity(b);
    const double E0 = orbiterEnergy(b.state);
    const double tEnd = orbits * TWO_PI;
    double t = 0.0;
    KeplerRun r;
    while (t < tEnd) {
        ias.step(b.state, f, t, tEnd);
        r.maxEnergyError = std::max(r.maxEnergyError, std::abs((orbiterEnergy(b.state) - E0) / E0));
    }
    r.finalTimeError = t - tEnd;
    r.steps = ias.totalSteps();
    r.returnError = std::hypot(b.state.x[1] - s0.x[1], b.state.y[1] - s0.y[1]);
    const Elements el = stateToElements({b.state.x[1], b.state.y[1], 0.0}, {b.state.vx[1], b.state.vy[1], 0.0}, 1.0);
    double dM = el.M; // the start is at periapsis: M = 0
    dM -= TWO_PI * std::floor(dM / TWO_PI + 0.5);
    r.periodError = std::abs(dM) / (TWO_PI * orbits);
    return r;
}
}

TEST_SUITE("ias15 adaptive: Kepler")
{
    // plan 14: e = 0, 0.5, 0.9, mu = 1, a = 1, energy and period error < 1e-10 over 1000 orbits.
    // Measured (default epsilon 1e-9): max|dE/E| 8.7e-15 / 6.7e-15 / 2.8e-14; relative period error 0 / 3e-15 / 1.8e-14;
    // return error 2.9e-11 / 3.4e-11 / 4.9e-10 (the e = 0.9 orbit has a periapsis at 0.1, so a phase error shows up larger).

    TEST_CASE("e = 0: 1000 orbits")
    {
        const KeplerRun r = runKepler(0.0, 1000);
        CHECK(r.finalTimeError == 0.0);
        CHECK(r.maxEnergyError < 1e-12);
        CHECK(r.periodError < 1e-12);
        CHECK(r.returnError < 1e-9);
    }

    TEST_CASE("e = 0.5: 1000 orbits")
    {
        const KeplerRun r = runKepler(0.5, 1000);
        CHECK(r.finalTimeError == 0.0);
        CHECK(r.maxEnergyError < 1e-12);
        CHECK(r.periodError < 1e-12);
        CHECK(r.returnError < 1e-9);
    }

    TEST_CASE("e = 0.9: 1000 orbits")
    {
        const KeplerRun r = runKepler(0.9, 1000);
        CHECK(r.finalTimeError == 0.0);
        CHECK(r.maxEnergyError < 1e-12);
        CHECK(r.periodError < 1e-12);
        CHECK(r.returnError < 5e-9);
    }

    TEST_CASE("a tighter epsilon takes more steps and keeps the energy at roundoff")
    {
        const KeplerRun loose = runKepler(0.5, 20, 1e-9);
        const KeplerRun tight = runKepler(0.5, 20, 1e-12);
        const double ratio = static_cast<double>(tight.steps) / loose.steps;
        CHECK(ratio > 2.2); // measured 2.44: the step scales as epsilon^(1/7), so 1000x tighter is 10^(3/7) = 2.7x more steps
        CHECK(ratio < 3.0);
        CHECK(tight.maxEnergyError < 1e-13);    // measured 1.3e-15
        CHECK(loose.maxEnergyError < 1e-13);    // measured 1.8e-15
    }
}

namespace {
// Figure-eight choreography (Chenciner & Montgomery 2000; the digits are those of plan section 14)
Bodies figureEight()
{
    Bodies b;
    const double v3x = -0.93240737, v3y = -0.86473146;
    b.add(body(1.0, 0.97000436, -0.24308753, -v3x / 2, -v3y / 2));
    b.add(body(1.0, -0.97000436, 0.24308753, -v3x / 2, -v3y / 2));
    b.add(body(1.0, 0.0, 0.0, v3x, v3y));
    return b;
}
const double FIGURE_EIGHT_PERIOD = 6.32591398;
}

TEST_SUITE("ias15 adaptive: figure-eight")
{
    TEST_CASE("stays on the choreography for 100 periods, energy and angular momentum conserved")
    {
        Bodies b = figureEight();
        const BodyState s0 = b.state;
        IAS15 ias;
        const ForceFn f = gravity(b);
        const double E0 = energy(b.state, b.mu), L0 = angularMomentum(b.state, b.mu);
        double t = 0.0, maxdE = 0.0, maxdev = 0.0, maxdL = 0.0;
        for (int period = 1; period <= 100; ++period) {
            const double tEnd = period * FIGURE_EIGHT_PERIOD;
            while (t < tEnd) {
                ias.step(b.state, f, t, tEnd);
                maxdE = std::max(maxdE, std::abs((energy(b.state, b.mu) - E0) / E0));
                maxdL = std::max(maxdL, std::abs(angularMomentum(b.state, b.mu) - L0));
            }
            double dev = 0.0; // every body is back near its own start after a whole period
            for (std::size_t i = 0; i < 3; ++i) dev = std::max(dev, std::hypot(b.state.x[i] - s0.x[i], b.state.y[i] - s0.y[i]));
            maxdev = std::max(maxdev, dev);
        }
        // measured: dE 1.2e-15, dL 4e-16. The return error grows linearly (4.1e-8 after 1 period, 4.0e-6 after 100)
        // because the initial data and the period have only 8 digits; it is the same at epsilon 1e-9 and 1e-11.
        CHECK(maxdE < 1e-12);
        CHECK(maxdL < 1e-12);
        CHECK(maxdev < 1e-5);
    }

    TEST_CASE("after T/3 every body sits where another one started (one orbit, time-shifted by T/3)")
    {
        Bodies b = figureEight();
        const BodyState s0 = b.state;
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        const double tEnd = FIGURE_EIGHT_PERIOD / 3.0;
        while (t < tEnd) ias.step(b.state, f, t, tEnd);
        // after T/3 each body sits where one of the others started; all three distinct
        int hit[3] = {-1, -1, -1};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                if (std::hypot(b.state.x[i] - s0.x[j], b.state.y[i] - s0.y[j]) < 1e-5) hit[i] = static_cast<int>(j);
        for (int i = 0; i < 3; ++i) CHECK(hit[i] >= 0);
        CHECK(hit[0] != hit[1]);
        CHECK(hit[1] != hit[2]);
        CHECK(hit[0] != hit[2]);
        for (int i = 0; i < 3; ++i) CHECK(hit[i] != i);
    }
}
