#include <doctest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"
#include "core/ias15.h"

using namespace cosmos;

// Non-finite input and output of the adaptive IAS15: a NaN or infinity is never committed. step() returns 0, leaves
// s and t exactly as they were, and lastFailed() says so (plan sections 6.3, 6.6).

namespace {
const double NaN = std::numeric_limits<double>::quiet_NaN();
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

bool allFinite(const BodyState& s)
{
    for (std::size_t i = 0; i < s.x.size(); ++i) {
        if (!(std::isfinite(s.x[i]) && std::isfinite(s.y[i]) && std::isfinite(s.z[i]) && std::isfinite(s.vx[i]) &&
              std::isfinite(s.vy[i]) && std::isfinite(s.vz[i])))
            return false;
    }
    return true;
}

bool sameBits(const BodyState& p, const BodyState& q)
{
    const std::vector<double>* a[6] = {&p.x, &p.y, &p.z, &p.vx, &p.vy, &p.vz};
    const std::vector<double>* b[6] = {&q.x, &q.y, &q.z, &q.vx, &q.vy, &q.vz};
    for (int c = 0; c < 6; ++c) {
        if (a[c]->size() != b[c]->size()) return false;
        if (!a[c]->empty() && std::memcmp(a[c]->data(), b[c]->data(), a[c]->size() * sizeof(double)) != 0) return false;
    }
    return true;
}
} // namespace

TEST_SUITE("ias15 adaptive: non-finite input and output")
{
    TEST_CASE("two coincident massive bodies: the step is refused, nothing changes, lastFailed() is set")
    {
        Bodies b;
        b.add(body(1.0, 0.5, 0.25, 0.1, 0.2));
        b.add(body(1.0, 0.5, 0.25, -0.3, 0.0)); // same place: the force is 0 * inf = NaN
        const BodyState before = b.state;
        IAS15 ias;
        double t = 3.0;
        const double dt = ias.step(b.state, gravity(b), t, 10.0);
        CHECK(dt == 0.0);
        CHECK(ias.lastFailed());
        CHECK(t == 3.0);
        CHECK(sameBits(b.state, before));
        CHECK(allFinite(b.state));
        CHECK(ias.totalSteps() == 0);
        CHECK_FALSE(ias.hasDenseOutput());
        // asking again changes nothing and keeps reporting the failure
        CHECK(ias.step(b.state, gravity(b), t, 10.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(sameBits(b.state, before));
    }

    TEST_CASE("after the bodies are separated and reset() the integrator works again; lastFailed() is cleared")
    {
        Bodies b;
        b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
        b.add(body(1.0, 0.0, 0.0, 0.0, 0.5));
        IAS15 ias;
        double t = 0.0;
        CHECK(ias.step(b.state, gravity(b), t, 1.0) == 0.0);
        CHECK(ias.lastFailed());
        b.state.x[1] = 1.0;
        ias.reset();
        const double dt = ias.step(b.state, gravity(b), t, 1.0);
        CHECK(dt > 0.0);
        CHECK_FALSE(ias.lastFailed());
        CHECK(t > 0.0);
        CHECK(allFinite(b.state));
    }

    TEST_CASE("a NaN or infinity in the state is refused the same way and left untouched")
    {
        Bodies b;
        b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
        b.add(body(0.0, 1.0, 0.0, 0.0, 1.0));
        IAS15 ias;
        double t = 0.0;
        CHECK(ias.step(b.state, gravity(b), t, 0.1) > 0.0);
        CHECK_FALSE(ias.lastFailed());
        const double tGood = t;
        b.state.vy[1] = NaN;
        CHECK(ias.step(b.state, gravity(b), t, 1.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(t == tGood);
        CHECK(std::isnan(b.state.vy[1])); // not repaired, not spread to the other components
        CHECK(std::isfinite(b.state.x[1]));
        b.state.vy[1] = 1.0;
        b.state.x[1] = INF;
        CHECK(ias.step(b.state, gravity(b), t, 1.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(t == tGood);
    }

    TEST_CASE("a force that is NaN only for the large trial step is cured by the retry with a smaller step")
    {
        // x'' = -x, but the force callback returns NaN wherever |x| > 1.5. The first trial (b = 0, dt = 10) puts the
        // nodes at x = 1 - 50 h^2, far outside; a quarter of that step is still fine.
        Bodies b;
        b.add(body(0.0, 1.0, 0.0, 0.0, 0.0));
        const ForceFn f = [](const BodyState& s, Accel& a) {
            const std::size_t n = s.x.size();
            a.ax.assign(n, 0.0);
            a.ay.assign(n, 0.0);
            a.az.assign(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) a.ax[i] = std::abs(s.x[i]) > 1.5 ? NaN : -s.x[i];
        };
        IAS15 ias;
        ias.setNextDt(10.0);
        double t = 0.0;
        const double dt = ias.step(b.state, f, t, 100.0);
        CHECK(dt > 0.0);
        CHECK(dt < 10.0);
        CHECK_FALSE(ias.lastFailed());
        CHECK(ias.lastRejections() >= 1);
        CHECK(allFinite(b.state));
        CHECK(t == dt);
        CHECK(std::abs(b.state.x[0] - std::cos(t)) < 1e-3);
    }

    TEST_CASE("a force that is NaN at every node: the step is abandoned after the retries, nothing is committed")
    {
        Bodies b;
        b.add(body(0.0, 1.0, 0.0, 0.0, 0.0));
        int calls = 0;
        const ForceFn f = [&calls](const BodyState& s, Accel& a) {
            const std::size_t n = s.x.size();
            a.ax.assign(n, 0.0);
            a.ay.assign(n, 0.0);
            a.az.assign(n, 0.0);
            // the first call is the force at the start state; every later one (the nodes) is NaN
            if (calls++ > 0) a.ay.assign(n, NaN);
            else a.ax[0] = -s.x[0];
        };
        IAS15 ias;
        const BodyState before = b.state;
        double t = 0.0;
        CHECK(ias.step(b.state, f, t, 5.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(ias.lastRejections() > 10);
        CHECK(t == 0.0);
        CHECK(sameBits(b.state, before));
        CHECK(ias.totalSteps() == 0);
        CHECK_FALSE(ias.hasDenseOutput());
        CHECK(ias.nextDt() > 0.0);
        CHECK(ias.nextDt() < 1.0); // the next call would try a (much) shorter step
    }

    TEST_CASE("dtMin: the retries stop at dtMin and the failure is reported")
    {
        Bodies b;
        b.add(body(0.0, 1.0, 0.0, 0.0, 0.0));
        int calls = 0;
        const ForceFn f = [&calls](const BodyState& s, Accel& a) {
            const std::size_t n = s.x.size();
            a.ax.assign(n, 0.0);
            a.ay.assign(n, 0.0);
            a.az.assign(n, 0.0);
            if (calls++ > 0) a.ax.assign(n, INF);
            else a.ax[0] = -s.x[0];
        };
        IAS15Params p;
        p.dtMin = 0.01;
        IAS15 ias(p);
        ias.setNextDt(1.0);
        double t = 0.0;
        CHECK(ias.step(b.state, f, t, 5.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(ias.lastRejections() <= 4); // 1 -> 0.25 -> 0.0625 -> 0.015625 -> dtMin
        CHECK(t == 0.0);
        CHECK(b.state.x[0] == 1.0);
    }

    TEST_CASE("lastFailed() is per call: it clears on a good step and on a call that does nothing")
    {
        Bodies b;
        b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
        b.add(body(0.0, 1.0, 0.0, 0.0, 1.0));
        IAS15 ias;
        double t = 0.0;
        CHECK(ias.step(b.state, gravity(b), t, 1.0) > 0.0);
        CHECK_FALSE(ias.lastFailed());
        b.state.x[1] = b.state.x[0]; // put the orbiter on top of the central body
        b.state.y[1] = b.state.y[0];
        ias.reset();
        CHECK(ias.step(b.state, gravity(b), t, 2.0) == 0.0);
        CHECK(ias.lastFailed());
        CHECK(ias.step(b.state, gravity(b), t, t) == 0.0); // t >= tMax: nothing attempted
        CHECK_FALSE(ias.lastFailed());
    }

    TEST_CASE("radial e = 1 infall into the singularity: every committed state is finite, a failure leaves the last good state")
    {
        // A test particle (mu = 0) falls from r = 1 at rest toward a unit mass; free-fall time pi / (2 sqrt 2) = 1.1107.
        // The integrator may get arbitrarily close, but it must never hand back a NaN or infinity.
        Bodies b;
        b.add(body(1.0, 0.0, 0.0, 0.0, 0.0));
        b.add(body(0.0, 1.0, 0.0, 0.0, 0.0));
        IAS15 ias;
        double t = 0.0;
        const double tMax = 2.0;
        long calls = 0;
        bool failed = false;
        double lastT = 0.0;
        while (t < tMax && calls < 2000000) {
            ++calls;
            const BodyState before = b.state;
            const double tBefore = t;
            const double dt = ias.step(b.state, gravity(b), t, tMax);
            REQUIRE(allFinite(b.state));
            REQUIRE(t >= lastT);
            lastT = t;
            if (ias.lastFailed()) {
                failed = true;
                CHECK(dt == 0.0);
                CHECK(t == tBefore);
                CHECK(sameBits(b.state, before));
                break;
            }
            REQUIRE(dt > 0.0);
        }
        // Measured: 7276 calls; it reaches r = 2e-103 at t = 1.11072 (the free-fall time pi / (2 sqrt 2) = 1.110721), where
        // the force overflows; no step is possible any more, so it stops there with lastFailed() set and the last good state.
        CHECK(failed);
        CHECK(std::abs(t - 3.14159265358979323846 / (2.0 * std::sqrt(2.0))) < 1e-3);
        CHECK(std::abs(b.state.x[1]) < 1e-6);
        CHECK(calls < 100000);
        CHECK(allFinite(b.state));
    }
}
