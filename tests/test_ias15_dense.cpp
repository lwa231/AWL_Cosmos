#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"
#include "core/ias15.h"
#include "core/kepler.h"

using namespace cosmos;

// Dense output, tMax clipping, reset, statistics and robustness of the adaptive IAS15 (plan sections 6.3, 6.6).
// Tolerances are set from measured values (the comment next to each says what was measured).

namespace {
const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const double INF = std::numeric_limits<double>::infinity();

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

Bodies keplerPair(double e)
{
    Bodies b;
    b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
    b.add(body(0.0, 1.0 - e, 0.0, 0.0, std::sqrt((1.0 + e) / (1.0 - e))));
    return b;
}

// Specific energy of the massless orbiter (body 1) around body 0
double orbiterEnergy(const BodyState& s, double mu0)
{
    return 0.5 * (s.vx[1] * s.vx[1] + s.vy[1] * s.vy[1]) - mu0 / std::hypot(s.x[1] - s.x[0], s.y[1] - s.y[0]);
}

Bodies pythagorean()
{
    Bodies b;
    b.add(body(3.0, 1.0, 3.0, 0.0, 0.0));
    b.add(body(4.0, -2.0, -1.0, 0.0, 0.0));
    b.add(body(5.0, 1.0, -1.0, 0.0, 0.0));
    return b;
}

bool sameBits(const BodyState& p, const BodyState& q)
{
    const std::size_t n = p.x.size();
    if (q.x.size() != n) return false;
    const std::vector<double>* a[6] = {&p.x, &p.y, &p.z, &p.vx, &p.vy, &p.vz};
    const std::vector<double>* b[6] = {&q.x, &q.y, &q.z, &q.vx, &q.vy, &q.vz};
    for (int c = 0; c < 6; ++c)
        if (std::memcmp(a[c]->data(), b[c]->data(), n * sizeof(double)) != 0) return false;
    return true;
}

double maxPositionDiff(const BodyState& p, const BodyState& q)
{
    double m = 0.0;
    for (std::size_t i = 0; i < p.x.size(); ++i) m = std::max(m, std::hypot(p.x[i] - q.x[i], p.y[i] - q.y[i]));
    return m;
}

double maxVelocityDiff(const BodyState& p, const BodyState& q)
{
    double m = 0.0;
    for (std::size_t i = 0; i < p.x.size(); ++i) m = std::max(m, std::hypot(p.vx[i] - q.vx[i], p.vy[i] - q.vy[i]));
    return m;
}

double maxAccel(const Accel& a)
{
    double m = 0.0;
    for (std::size_t i = 0; i < a.ax.size(); ++i) m = std::max(m, std::hypot(a.ax[i], a.ay[i]));
    return m;
}

// Take `steps` free steps from the initial state, then one more step whose dense output is compared, at `tau`, with a
// separate integration (a fresh integrator from the start of that step) that is made to land exactly on that time.
struct InteriorResult {
    double positionError = 0.0; // max over bodies, relative to the largest distance from the origin
    double velocityError = 0.0; // relative to the largest speed
};

InteriorResult interiorVersusLanding(Bodies b, int steps, double tau, double epsilon = 1e-9, double epsilonReference = 1e-9)
{
    IAS15Params p;
    p.epsilon = epsilon;
    IAS15 ias(p);
    const ForceFn f = gravity(b);
    double t = 0.0;
    for (int k = 0; k < steps; ++k) ias.step(b.state, f, t);
    const BodyState s0 = b.state;
    const double t0 = t;
    ias.step(b.state, f, t);
    const double ts = ias.stepStartTime() + tau * ias.lastDt();
    BodyState dense;
    ias.stateAt(tau, dense);

    Bodies c = b;
    c.state = s0;
    IAS15Params pr;
    pr.epsilon = epsilonReference;
    IAS15 other(pr);
    const ForceFn g = gravity(c);
    double t2 = t0;
    while (t2 < ts) other.step(c.state, g, t2, ts);

    double scaleX = 0.0, scaleV = 0.0;
    for (std::size_t i = 0; i < c.size(); ++i) {
        scaleX = std::max(scaleX, std::hypot(c.state.x[i], c.state.y[i]));
        scaleV = std::max(scaleV, std::hypot(c.state.vx[i], c.state.vy[i]));
    }
    InteriorResult r;
    r.positionError = maxPositionDiff(dense, c.state) / scaleX;
    r.velocityError = maxVelocityDiff(dense, c.state) / scaleV;
    return r;
}
}

TEST_SUITE("ias15 adaptive: dense output")
{
    TEST_CASE("tau = 0 reproduces the start of the step exactly, tau = 1 its end; accelerations likewise")
    {
        Bodies b = pythagorean();
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        CHECK_FALSE(ias.hasDenseOutput());
        double worstEnd = 0.0, worstEndAcc = 0.0;
        for (int k = 0; k < 40; ++k) {
            const BodyState before = b.state;
            Accel aBefore;
            computeAccelerations(before, b.mu, aBefore);
            const double dt = ias.step(b.state, f, t);
            CHECK(ias.hasDenseOutput());
            CHECK(ias.lastDt() == dt);
            CHECK(ias.stepEndTime() == t);
            CHECK(ias.bodyCount() == 3);

            BodyState atStart, atEnd;
            ias.stateAt(0.0, atStart);
            CHECK(sameBits(atStart, before));
            Accel accStart;
            ias.accelerationsAt(0.0, accStart);
            for (std::size_t i = 0; i < 3; ++i) {
                CHECK(accStart.ax[i] == aBefore.ax[i]);
                CHECK(accStart.ay[i] == aBefore.ay[i]);
            }

            ias.stateAt(1.0, atEnd);
            worstEnd = std::max(worstEnd, std::max(maxPositionDiff(atEnd, b.state), maxVelocityDiff(atEnd, b.state)));
            Accel accEnd, aAfter;
            ias.accelerationsAt(1.0, accEnd);
            computeAccelerations(b.state, b.mu, aAfter);
            double d = 0.0;
            for (std::size_t i = 0; i < 3; ++i) d = std::max(d, std::hypot(accEnd.ax[i] - aAfter.ax[i], accEnd.ay[i] - aAfter.ay[i]));
            worstEndAcc = std::max(worstEndAcc, d / maxAccel(aAfter));
        }
        CHECK(worstEnd < 1e-13);    // measured 4.6e-16: the polynomial and the Kahan-summed state agree to roundoff
        CHECK(worstEndAcc < 1e-9);  // measured 5e-14: a(1) is an extrapolation of the b polynomial, error ~ the neglected b7 term
    }

    TEST_CASE("interior of a step matches a separate integration that lands exactly on that time (Kepler e = 0.5)")
    {
        double worstX = 0.0, worstV = 0.0;
        for (double tau : {0.1, 0.37, 0.5, 0.9}) {
            const InteriorResult r = interiorVersusLanding(keplerPair(0.5), 10, tau);
            worstX = std::max(worstX, r.positionError);
            worstV = std::max(worstV, r.velocityError);
        }
        CHECK(worstX < 1e-10); // plan 14: about 1e-10 relative. Measured below 2e-16 (steps this small are exact to roundoff)
        CHECK(worstV < 1e-10);
    }

    TEST_CASE("interior of a step matches a separate integration (Pythagorean problem, strongly interacting)")
    {
        double worstX = 0.0, worstV = 0.0;
        for (int steps : {5, 60, 200}) {
            const InteriorResult r = interiorVersusLanding(pythagorean(), steps, 0.37);
            worstX = std::max(worstX, r.positionError);
            worstV = std::max(worstV, r.velocityError);
        }
        CHECK(worstX < 1e-11); // measured 8e-16 (position) and 1.7e-13 (velocity, near a close encounter)
        CHECK(worstV < 1e-11);
    }

    TEST_CASE("with coarse steps (epsilon 0.1) the interpolation error becomes visible but stays small")
    {
        // The default-epsilon steps are so small that the dense output is exact to roundoff (errors below). Coarse steps
        // show its real error against a precise reference (epsilon 1e-12).
        double worstX = 0.0, worstV = 0.0;
        for (double tau : {0.25, 0.5, 0.75}) {
            const InteriorResult r = interiorVersusLanding(keplerPair(0.5), 6, tau, 1e-1, 1e-12);
            worstX = std::max(worstX, r.positionError);
            worstV = std::max(worstV, r.velocityError);
        }
        CHECK(worstX < 1e-9);  // measured 1.0e-10
        CHECK(worstV < 5e-8);  // measured 4.9e-9: the velocity polynomial is one order lower in dt
        CHECK(worstV > 1e-11); // and the comparison really sees an interpolation error (a test that always says 0 tests nothing)
    }

    TEST_CASE("accelerations from the polynomial match the force at the interpolated positions")
    {
        Bodies b = pythagorean();
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 30; ++k) ias.step(b.state, f, t);
        double worst = 0.0;
        for (double tau : {0.1, 0.25, 0.5, 0.75, 0.95}) {
            BodyState s;
            ias.stateAt(tau, s);
            Accel poly, direct;
            ias.accelerationsAt(tau, poly);
            computeAccelerations(s, b.mu, direct);
            double d = 0.0;
            for (std::size_t i = 0; i < 3; ++i) d = std::max(d, std::hypot(poly.ax[i] - direct.ax[i], poly.ay[i] - direct.ay[i]));
            worst = std::max(worst, d / maxAccel(direct));
        }
        CHECK(worst < 1e-10); // measured 4.7e-15
    }

    TEST_CASE("velocity is the derivative of position along the step, acceleration the derivative of velocity")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 7; ++k) ias.step(b.state, f, t);
        const double dt = ias.lastDt(), h = 1e-5;
        for (double tau : {0.2, 0.5, 0.8}) {
            const Vec3 pPlus = ias.positionOf(1, tau + h), pMinus = ias.positionOf(1, tau - h);
            const Vec3 vPlus = ias.velocityOf(1, tau + h), vMinus = ias.velocityOf(1, tau - h);
            const Vec3 v = ias.velocityOf(1, tau), a = ias.accelerationOf(1, tau);
            const double fdV = std::hypot((pPlus.x - pMinus.x) / (2 * h * dt) - v.x, (pPlus.y - pMinus.y) / (2 * h * dt) - v.y);
            const double fdA = std::hypot((vPlus.x - vMinus.x) / (2 * h * dt) - a.x, (vPlus.y - vMinus.y) / (2 * h * dt) - a.y);
            CHECK(fdV < 1e-7);
            CHECK(fdA < 1e-6);
        }
    }

    TEST_CASE("single-body queries agree with the whole-system ones; time-based tau")
    {
        Bodies b = pythagorean();
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 12; ++k) ias.step(b.state, f, t);
        BodyState s;
        ias.stateAt(0.4, s);
        Accel a;
        ias.accelerationsAt(0.4, a);
        for (std::size_t i = 0; i < 3; ++i) {
            const Vec3 p = ias.positionOf(i, 0.4), v = ias.velocityOf(i, 0.4), acc = ias.accelerationOf(i, 0.4);
            CHECK(p.x == s.x[i]);
            CHECK(p.y == s.y[i]);
            CHECK(v.x == s.vx[i]);
            CHECK(v.y == s.vy[i]);
            CHECK(acc.x == a.ax[i]);
            CHECK(acc.y == a.ay[i]);
        }
        const double time = ias.stepStartTime() + 0.4 * ias.lastDt();
        CHECK(ias.tauAt(time) == doctest::Approx(0.4).epsilon(1e-12));
        CHECK(ias.tauAt(ias.stepStartTime()) == 0.0);
        CHECK(ias.tauAt(ias.stepEndTime()) == doctest::Approx(1.0).epsilon(1e-12));
    }

    TEST_CASE("after a step that needed rejections the dense output is that of the accepted step; reset clears it")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        ias.setNextDt(20.0); // too big: the first attempt is rejected
        const BodyState before = b.state;
        const double dt = ias.step(b.state, f, t);
        REQUIRE(ias.lastRejections() > 0);
        CHECK(ias.lastDt() == dt);
        CHECK(ias.stepStartTime() == 0.0);
        CHECK(ias.stepEndTime() == t);
        BodyState atStart, atEnd;
        ias.stateAt(0.0, atStart);
        ias.stateAt(1.0, atEnd);
        CHECK(sameBits(atStart, before));
        CHECK(maxPositionDiff(atEnd, b.state) < 1e-13);
        ias.reset();
        CHECK_FALSE(ias.hasDenseOutput());
    }
}

TEST_SUITE("ias15 adaptive: step control")
{
    TEST_CASE("tMax: the step is clipped to land exactly, never beyond")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 5; ++k) ias.step(b.state, f, t);
        const double t0 = t;
        const double tMax = t0 + 0.0123456789;
        const double dt = ias.step(b.state, f, t, tMax);
        CHECK(t == tMax); // bit exact
        CHECK(dt == doctest::Approx(0.0123456789).epsilon(1e-12));
        CHECK(ias.lastDt() == dt);
        CHECK(ias.stepEndTime() == tMax);

        // many different clip distances, including the awkward ones
        for (double gap : {1e-9, 3.3e-4, 0.05, 0.4, 30.0}) {
            const double tm = t + gap;
            while (t < tm) {
                const double before = t;
                ias.step(b.state, f, t, tm);
                CHECK(t <= tm);
                CHECK(t > before);
            }
            CHECK(t == tm);
        }
    }

    TEST_CASE("tMax at or before t: nothing happens")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 1.0;
        const BodyState before = b.state;
        CHECK(ias.step(b.state, f, t, 1.0) == 0.0);
        CHECK(ias.step(b.state, f, t, 0.5) == 0.0);
        CHECK(t == 1.0);
        CHECK(sameBits(b.state, before));
        CHECK_FALSE(ias.hasDenseOutput());
        CHECK(ias.totalSteps() == 0);
    }

    TEST_CASE("a step clipped to a tiny interval does not make the next proposal tiny")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 10; ++k) ias.step(b.state, f, t);
        const double proposal = ias.nextDt();
        CHECK(proposal > 0.0);
        ias.step(b.state, f, t, t + proposal * 1e-4);
        CHECK(ias.nextDt() > 0.5 * proposal);
        CHECK(ias.nextDt() < 2.0 * proposal);
    }

    TEST_CASE("landing on thousands of sample times costs no accuracy (Kepler e = 0.5, 50 orbits, a sample every 0.1)")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        const double E0 = orbiterEnergy(b.state, 1.0);
        double t = 0.0, maxdE = 0.0;
        const int samples = static_cast<int>(50 * TWO_PI / 0.1);
        for (int k = 1; k <= samples; ++k) {
            const double tm = k * 0.1;
            while (t < tm) ias.step(b.state, f, t, tm);
            maxdE = std::max(maxdE, std::abs((orbiterEnergy(b.state, 1.0) - E0) / E0));
        }
        CHECK(maxdE < 1e-13); // measured 1.8e-15 (7286 steps for the 3141 samples)
    }

    TEST_CASE("an oversized first step is rejected and retried, the result is still right")
    {
        Bodies b = keplerPair(0.0);
        IAS15 ias;
        const ForceFn f = gravity(b);
        const double E0 = orbiterEnergy(b.state, 1.0);
        ias.setNextDt(20.0); // three orbits in one step
        double t = 0.0;
        ias.step(b.state, f, t);
        CHECK(ias.lastRejections() > 0);
        CHECK(ias.totalRejections() == ias.lastRejections());
        CHECK(ias.lastDt() < 20.0 * 0.25 + 1e-12); // a rejection means the proposal shrank to below a quarter
        const double tEnd = 20 * TWO_PI;
        while (t < tEnd) ias.step(b.state, f, t, tEnd);
        CHECK(std::abs((orbiterEnergy(b.state, 1.0) - E0) / E0) < 1e-12);
    }

    TEST_CASE("the step grows by at most 4x from one step to the next")
    {
        Bodies b = keplerPair(0.0);
        IAS15 ias;
        const ForceFn f = gravity(b);
        ias.setNextDt(1e-9); // far too small
        double t = 0.0, prev = 0.0, maxGrowth = 0.0;
        int steps = 0;
        while (steps < 60) {
            const double dt = ias.step(b.state, f, t);
            if (prev > 0.0) maxGrowth = std::max(maxGrowth, dt / prev);
            prev = dt;
            ++steps;
        }
        CHECK(maxGrowth <= 4.0 * (1.0 + 1e-12));
        CHECK(maxGrowth > 2.5);  // measured 3.37: it grows nearly at the maximum rate while the steps are far too small
        CHECK(prev > 0.05);      // 60 steps from 1e-9 are enough to reach the natural step (~0.1 for a circular orbit)
    }

    TEST_CASE("dtMax and dtMin bound the proposed steps")
    {
        Bodies b = keplerPair(0.0);
        IAS15Params p;
        p.dtMax = 0.01;
        IAS15 ias(p);
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 50; ++k) CHECK(ias.step(b.state, f, t) <= 0.01);
        CHECK(ias.nextDt() <= 0.01);
    }

    TEST_CASE("statistics: iterations, rejections, step counts")
    {
        Bodies b = keplerPair(0.5);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 1000; ++k) {
            const double dt = ias.step(b.state, f, t);
            CHECK(ias.lastDt() == dt);
            CHECK(ias.nextDt() > 0.0);
            CHECK(ias.nextDt() <= 4.0 * dt * (1.0 + 1e-12));
            CHECK(ias.lastIterations() >= 1);
            CHECK(ias.lastIterations() <= 12);
        }
        CHECK(ias.totalSteps() == 1000);
        const double perStep = static_cast<double>(ias.totalIterations()) / 1000.0;
        CHECK(perStep > 1.0);
        CHECK(perStep < 3.5); // measured about 3.0: two or three Gauss-Seidel sweeps with the extrapolated predictor
        CHECK(ias.forceCalls() == 1 + 1000 + 7 * ias.totalIterations()); // initial a, one per step at the new positions, 7 per sweep
    }

    TEST_CASE("time and positions are summed with compensation: 100000 steps of 0.1 reach 10000 to within 1e-11")
    {
        Bodies b;
        b.add(body(0.0, 0.0, 0.0, 1.0, 0.0));
        b.add(body(0.0, 1.0, 1.0, 0.0, 1.0));
        IAS15Params p;
        p.dtMax = 0.1;
        IAS15 ias(p);
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 100000; ++k) ias.step(b.state, f, t);
        CHECK(std::abs(t - 10000.0) < 1e-11);
        CHECK(std::abs(b.state.x[0] - 10000.0) < 1e-11); // body 0 moves at speed 1: the position sum is compensated too
        CHECK(ias.totalRejections() == 0);
    }

    TEST_CASE("no forces at all: free flight, exact, no NaN, lands on tMax in one or a few steps")
    {
        Bodies b;
        b.add(body(0.0, 0.0, 0.0, 1.0, 2.0));
        b.add(body(0.0, 5.0, 5.0, -1.0, 0.5));
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        while (t < 8.0) ias.step(b.state, f, t, 8.0);
        CHECK(t == 8.0);
        CHECK(b.state.x[0] == doctest::Approx(8.0).epsilon(1e-15));
        CHECK(b.state.y[0] == doctest::Approx(16.0).epsilon(1e-15));
        CHECK(b.state.x[1] == doctest::Approx(-3.0).epsilon(1e-15));
        CHECK(b.state.y[1] == doctest::Approx(9.0).epsilon(1e-15));
        CHECK(ias.totalRejections() == 0);
        CHECK(ias.totalSteps() <= 3);
    }

    TEST_CASE("currentAccelerations() is the force at the state the last step left")
    {
        Bodies b = pythagorean();
        IAS15 ias;
        const ForceFn f = gravity(b);
        CHECK(ias.currentAccelerations().ax.empty());
        double t = 0.0;
        for (int k = 0; k < 10; ++k) {
            ias.step(b.state, f, t);
            Accel ref;
            computeAccelerations(b.state, b.mu, ref);
            for (std::size_t i = 0; i < 3; ++i) {
                CHECK(ias.currentAccelerations().ax[i] == ref.ax[i]);
                CHECK(ias.currentAccelerations().ay[i] == ref.ay[i]);
            }
        }
        ias.reset();
        CHECK(ias.currentAccelerations().ax.empty());
    }

    TEST_CASE("an empty system does nothing")
    {
        BodyState s;
        IAS15 ias;
        const ForceFn f = [](const BodyState&, Accel& a) { a.ax.clear(); a.ay.clear(); a.az.clear(); };
        double t = 0.0;
        CHECK(ias.step(s, f, t, 1.0) == 0.0);
        CHECK(t == 0.0);
    }
}

TEST_SUITE("ias15 adaptive: reset and determinism")
{
    TEST_CASE("after a mass changes behind its back, reset() makes the integrator re-evaluate the accelerations")
    {
        Bodies b = keplerPair(0.3);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 20; ++k) ias.step(b.state, f, t);
        b.mu[0] = 1.3; // e.g. a merge changed the mass
        ias.reset();
        CHECK_FALSE(ias.hasDenseOutput());
        CHECK(ias.nextDt() == 0.0);
        const double E0 = orbiterEnergy(b.state, 1.3);
        double maxdE = 0.0;
        const double tEnd = t + 10 * TWO_PI;
        const double dtFirst = ias.step(b.state, f, t, tEnd);
        CHECK(dtFirst > 1e-4); // a stale cached acceleration would look like a jump in a(t) and shrink the first step to ~1e-18
        while (t < tEnd) {
            ias.step(b.state, f, t, tEnd);
            maxdE = std::max(maxdE, std::abs((orbiterEnergy(b.state, 1.3) - E0) / E0));
        }
        CHECK(maxdE < 1e-12); // measured 9e-16
    }

    TEST_CASE("a changed number of bodies is noticed: history is dropped, the new body is integrated correctly")
    {
        Bodies b = keplerPair(0.2);
        IAS15 ias;
        const ForceFn f = gravity(b);
        double t = 0.0;
        for (int k = 0; k < 20; ++k) ias.step(b.state, f, t);
        BodyDesc probe = body(0.0, 0.0, 3.0, std::sqrt(1.0 / 3.0), 0.0); // circular orbit around body 0 at r = 3
        b.add(probe);
        CHECK(ias.step(b.state, f, t) > 0.0);
        CHECK(ias.bodyCount() == 3);
        const double r0 = std::hypot(b.state.x[2], b.state.y[2]);
        const double tEnd = t + 5 * TWO_PI * std::pow(3.0, 1.5);
        while (t < tEnd) ias.step(b.state, f, t, tEnd);
        CHECK(std::abs(std::hypot(b.state.x[2], b.state.y[2]) - r0) < 1e-9);
        // removing a body
        b.remove(b.ids[2]);
        CHECK(ias.step(b.state, f, t) > 0.0);
        CHECK(ias.bodyCount() == 2);
    }

    TEST_CASE("deterministic: the same inputs give bit-identical states, also across reset")
    {
        auto run = [](bool resetMidway) {
            Bodies b = pythagorean();
            IAS15 ias;
            const ForceFn f = gravity(b);
            double t = 0.0;
            while (t < 10.0) ias.step(b.state, f, t, 10.0);
            if (resetMidway) ias.reset();
            while (t < 12.0) ias.step(b.state, f, t, 12.0);
            return b.state;
        };
        const BodyState a = run(false), c = run(false);
        CHECK(sameBits(a, c));
        const BodyState d = run(true), e = run(true);
        CHECK(sameBits(d, e));
    }

    TEST_CASE("velocity-dependent forces: damped oscillator x'' = -x - 0.2 x', node velocities reach the callback")
    {
        const double gamma = 0.2;
        const double w = std::sqrt(1.0 - gamma * gamma / 4.0);
        BodyState s;
        s.x = {1.0}; s.y = {0.0}; s.z = {0.0};
        s.vx = {0.0}; s.vy = {0.0}; s.vz = {0.0};
        const ForceFn f = [gamma](const BodyState& st, Accel& a) {
            a.ax.assign(1, -st.x[0] - gamma * st.vx[0]);
            a.ay.assign(1, 0.0);
            a.az.assign(1, 0.0);
        };
        IAS15 ias;
        double t = 0.0;
        const double tEnd = 40.0;
        while (t < tEnd) ias.step(s, f, t, tEnd);
        const double decay = std::exp(-gamma * tEnd / 2.0);
        const double xExact = decay * (std::cos(w * tEnd) + gamma / (2.0 * w) * std::sin(w * tEnd));
        const double vExact = -decay * (1.0 / w) * std::sin(w * tEnd); // x' = -e^(-g t/2) sin(w t) / w for x(0) = 1, v(0) = 0
        CHECK(std::abs(s.x[0] - xExact) < 1e-9);
        CHECK(std::abs(s.vx[0] - vExact) < 1e-9);
    }
}

namespace {
// Two bodies (mu 4 and 5, a = 0.01, e = 0.5, starting at periapsis) whose centre of mass sits at (offset, 0)
Bodies farTightBinary(double offset)
{
    Elements el;
    el.a = 0.01;
    el.e = 0.5;
    el.omega = 0.0;
    el.M = 0.0;
    const RelState rel = elementsToState(el, 9.0);
    Bodies b;
    b.add(body(4.0, offset - 5.0 / 9.0 * rel.r.x, -5.0 / 9.0 * rel.r.y, -5.0 / 9.0 * rel.v.x, -5.0 / 9.0 * rel.v.y));
    b.add(body(5.0, offset + 4.0 / 9.0 * rel.r.x, 4.0 / 9.0 * rel.r.y, 4.0 / 9.0 * rel.v.x, 4.0 / 9.0 * rel.v.y));
    return b;
}

struct BinaryRun {
    long steps = 0;
    double tReached = 0.0;
    double maxEnergyError = 0.0;
    double lastEpsilon = 0.0;
};

BinaryRun runBinary(double offset, double noiseFactor, long stepBudget, int periods)
{
    Bodies b = farTightBinary(offset);
    IAS15Params p;
    p.noiseFactor = noiseFactor;
    IAS15 ias(p);
    const ForceFn f = gravity(b);
    const double period = TWO_PI * std::sqrt(1e-6 / 9.0);
    const double tEnd = periods * period;
    auto binding = [&]() {
        const double vx = b.state.vx[1] - b.state.vx[0], vy = b.state.vy[1] - b.state.vy[0];
        return 0.5 * (vx * vx + vy * vy) - 9.0 / std::hypot(b.state.x[1] - b.state.x[0], b.state.y[1] - b.state.y[0]);
    };
    const double E0 = binding();
    BinaryRun r;
    double t = 0.0;
    while (t < tEnd && ias.totalSteps() < stepBudget) {
        ias.step(b.state, f, t, tEnd);
        r.maxEnergyError = std::max(r.maxEnergyError, std::abs((binding() - E0) / E0));
    }
    r.steps = ias.totalSteps();
    r.tReached = t / period;
    r.lastEpsilon = ias.lastEpsilon();
    return r;
}
}

TEST_SUITE("ias15 adaptive: roundoff guard")
{
    TEST_CASE("a tight pair far from the origin: the step does not collapse")
    {
        // The pair is 0.01 apart at x = 3000: positions are rounded to 4.5e-13, so the force is only known to ~1e-10 relative
        // and the b6 error estimate cannot reach epsilon = 1e-9 however small the step. Without the guard the step shrinks
        // for ever; with it the target is raised just enough.
        const BinaryRun guarded = runBinary(3000.0, 1e4, 100000, 50);
        const BinaryRun unguarded = runBinary(3000.0, 0.0, 100000, 50);
        // measured: guarded 2309 steps for 50 periods, target raised to 1.3e-6, binding energy error 7e-10;
        // unguarded: 100000 steps (the budget) and not even one period done
        CHECK(guarded.tReached >= 50.0);
        CHECK(guarded.steps < 5000);
        CHECK(guarded.lastEpsilon > 1e-9);
        CHECK(guarded.maxEnergyError < 1e-8);
        CHECK(unguarded.tReached < 1.0);
    }

    TEST_CASE("near the origin the guard is inactive: epsilon is used as given")
    {
        const BinaryRun r = runBinary(0.0, 1e4, 100000, 20);
        CHECK(r.lastEpsilon == 1e-9);
        CHECK(r.tReached >= 20.0);
        CHECK(r.maxEnergyError < 1e-13); // measured 1.5e-15
    }
}
