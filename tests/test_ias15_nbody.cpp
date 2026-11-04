#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include "core/bodies.h"
#include "core/constants.h"
#include "core/gravity.h"
#include "core/ias15.h"
#include "core/kepler.h"

using namespace cosmos;

// Plan section 14: Pythagorean three-body problem and Sun-Jupiter-Saturn with the adaptive IAS15.
// Tolerances are set from measured values (the comment next to each says what was measured).

namespace {
const double PI = 3.14159265358979323846;

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

double distance(const BodyState& s, std::size_t i, std::size_t j) { return std::hypot(s.x[i] - s.x[j], s.y[i] - s.y[j]); }

// ---------------------------------------------------------------------------------------------------------------
// Pythagorean problem (Burrau 1913): mu = 3, 4, 5 at (1, 3), (-2, -1), (1, -1), at rest
// ---------------------------------------------------------------------------------------------------------------
struct PythagoreanRun {
    BodyState final;
    double maxEnergyError = 0.0;
    double maxMomentum = 0.0;
    double closestApproachTime = 0.0; // of body 0 (mu = 3) to either other body, deepest one after t = 50
    double closestApproach = 1e9;
    long steps = 0;
    long rejections = 0;
    double tReached = 0.0;
};

PythagoreanRun runPythagorean(double epsilon, double tEnd)
{
    Bodies b;
    b.add(body(3.0, 1.0, 3.0, 0.0, 0.0));
    b.add(body(4.0, -2.0, -1.0, 0.0, 0.0));
    b.add(body(5.0, 1.0, -1.0, 0.0, 0.0));
    IAS15Params p;
    p.epsilon = epsilon;
    IAS15 ias(p);
    const ForceFn f = gravity(b);
    const double E0 = energy(b.state, b.mu);
    PythagoreanRun r;
    double t = 0.0;
    while (t < tEnd && ias.totalSteps() < 300000) { // measured 25000 steps to t = 100: a stalled step must fail the test, not hang it
        ias.step(b.state, f, t, tEnd);
        r.maxEnergyError = std::max(r.maxEnergyError, std::abs((energy(b.state, b.mu) - E0) / E0));
        double px = 0.0, py = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            px += b.mu[i] * b.state.vx[i];
            py += b.mu[i] * b.state.vy[i];
        }
        r.maxMomentum = std::max(r.maxMomentum, std::hypot(px, py));
        if (t > 50.0) { // sample the end of the step and (dense output) the middle of it
            for (double tau : {0.5, 1.0}) {
                BodyState s;
                ias.stateAt(tau, s);
                const double d0 = std::min(distance(s, 0, 1), distance(s, 0, 2));
                if (d0 < r.closestApproach) {
                    r.closestApproach = d0;
                    r.closestApproachTime = ias.stepStartTime() + tau * ias.lastDt();
                }
            }
        }
    }
    r.final = b.state;
    r.tReached = t;
    r.steps = ias.totalSteps();
    r.rejections = ias.totalRejections();
    return r;
}

// Two-body energy per unit reduced mass. groupIsBC: body a against the centre of mass of the group (b, c), total
// mass mu_a + mu_b + mu_c. Otherwise: the pair (b, c) among themselves (a is unused).
double pairEnergy(const BodyState& s, const std::vector<double>& mu, std::size_t a, std::size_t b, std::size_t c, bool groupIsBC)
{
    if (groupIsBC) {
        const double mg = mu[b] + mu[c];
        const double gx = (mu[b] * s.x[b] + mu[c] * s.x[c]) / mg, gy = (mu[b] * s.y[b] + mu[c] * s.y[c]) / mg;
        const double gvx = (mu[b] * s.vx[b] + mu[c] * s.vx[c]) / mg, gvy = (mu[b] * s.vy[b] + mu[c] * s.vy[c]) / mg;
        const double r = std::hypot(s.x[a] - gx, s.y[a] - gy);
        const double v2 = (s.vx[a] - gvx) * (s.vx[a] - gvx) + (s.vy[a] - gvy) * (s.vy[a] - gvy);
        return 0.5 * v2 - (mu[a] + mg) / r;
    }
    const double r = distance(s, b, c);
    const double v2 = (s.vx[b] - s.vx[c]) * (s.vx[b] - s.vx[c]) + (s.vy[b] - s.vy[c]) * (s.vy[b] - s.vy[c]);
    return 0.5 * v2 - (mu[b] + mu[c]) / r;
}
}

TEST_SUITE("ias15 adaptive: Pythagorean three-body")
{
    TEST_CASE("mu = 3 escapes, mu = 4 and 5 leave as a bound binary, last close approach near t = 59.4")
    {
        const std::vector<double> mu = {3.0, 4.0, 5.0};
        const PythagoreanRun r = runPythagorean(1e-9, 100.0);
        REQUIRE(r.tReached == 100.0);
        std::printf("[validation] Pythagorean: E(body 0 vs binary) %.4g, E(binary) %.4g, separation at t=100 %.4g, last close approach %.4g at t = %.4f, %ld steps\n",
                    pairEnergy(r.final, mu, 0, 1, 2, true), pairEnergy(r.final, mu, 0, 1, 2, false),
                    std::hypot(r.final.x[0] - r.final.x[1], r.final.y[0] - r.final.y[1]), r.closestApproach, r.closestApproachTime, r.steps);
        CHECK(r.steps < 40000); // measured 25165
        const double eEscape = pairEnergy(r.final, mu, 0, 1, 2, true);
        const double eBinary = pairEnergy(r.final, mu, 0, 1, 2, false);
        const double sep0 = std::hypot(r.final.x[0] - r.final.x[1], r.final.y[0] - r.final.y[1]);
        CHECK(eEscape > 1.0);   // body 0 against the binary's centre of mass: unbound (measured +2.36)
        CHECK(eBinary < -1.0);  // bodies 1 and 2 against each other: bound (measured -8.15, semi-major axis 0.55)
        CHECK(sep0 > 50.0);     // and far apart by t = 100 (measured 97)
        CHECK(distance(r.final, 1, 2) < 1.2); // the binary stays within its apocentre, 2a = 1.1 (measured 0.73 at t = 100)
        // the deepest approach of body 0 to either other body after t = 50 is the encounter that ejects it:
        // measured 0.4725 at t = 59.420 (plan 14: t ~ 59.4)
        CHECK(r.closestApproachTime > 59.3);
        CHECK(r.closestApproachTime < 59.5);
        CHECK(r.closestApproach < 0.6);
    }

    TEST_CASE("the same outcome at epsilon 1e-11: close approach time agrees to 0.005")
    {
        const PythagoreanRun a = runPythagorean(1e-9, 70.0);
        const PythagoreanRun b = runPythagorean(1e-11, 70.0);
        CHECK(std::abs(a.closestApproachTime - b.closestApproachTime) < 0.005); // measured: identical to 4 decimals (59.4203)
        CHECK(std::abs(a.closestApproach - b.closestApproach) < 0.01);
    }

    TEST_CASE("energy stays within 1e-8 through the close encounters; momentum stays at roundoff")
    {
        const PythagoreanRun r = runPythagorean(1e-9, 70.0);
        CHECK(r.maxEnergyError < 1e-8);  // measured 9.2e-10 (5.9e-10 at epsilon 1e-11, where the roundoff guard sets the limit)
        CHECK(r.maxMomentum < 1e-12);    // measured 8e-14
    }
}
