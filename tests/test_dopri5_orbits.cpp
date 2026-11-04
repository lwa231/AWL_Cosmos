#include <doctest.h>
#include <cmath>
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Adaptive DOPRI5 accuracy on bound orbits around a point mass, through propagateShuttle (plan 6.4, 14).
// A single call covers the whole run, so the error control and the (here absent) operators are what is measured.

namespace {
struct Outcome {
    double relPosErr = 0.0;  // |r - r_exact| / a at the end
    double relEnergyDrift = 0.0;
    double stepsPerOrbit = 0.0;
    long rejected = 0;
};

// Shuttle on a Kepler orbit (a, e) about a point mass moving as r = origin + v t (a Galilean boost of the same orbit).
Outcome runOrbit(double mu, double a, double e, double orbits, const Vec3& origin, const Vec3& boost, double rtol)
{
    Bodies b = singleBody(mu, 0.0, origin);
    b.state.vx[0] = boost.x;
    b.state.vy[0] = boost.y;
    LinearEphemeris eph(b, 0.0);
    ControllerSet cs;
    ShuttleEnvironment env(b, eph, cs);
    env.tolerance.rtol = rtol;

    const double M0 = 0.7, omega = 0.3;
    const RelState st = keplerAt(mu, a, e, omega, M0, 0.0);
    Shuttles s;
    REQUIRE(s.add(shuttleDesc(1, origin + st.r, boost + st.v)));
    std::vector<Event> ev;
    ShuttleStats stats;
    const double T = orbits * keplerPeriod(mu, a);
    propagateShuttle(s, 0, 0.0, T, env, ev, nullptr, &stats);

    const RelState ref = keplerAt(mu, a, e, omega, M0, T);
    const Vec3 r = shuttlePos(s, 0) - (origin + boost * T);
    const Vec3 v = shuttleVel(s, 0) - boost;
    Outcome o;
    o.relPosErr = norm(r - ref.r) / a;
    const double E0 = 0.5 * norm2(st.v) - mu / norm(st.r);
    const double E1 = 0.5 * norm2(v) - mu / norm(r);
    o.relEnergyDrift = std::fabs((E1 - E0) / E0);
    o.stepsPerOrbit = static_cast<double>(stats.acceptedSteps) / orbits;
    o.rejected = stats.rejectedSteps;
    return o;
}
} // namespace

TEST_SUITE("dopri5 orbits")
{
    TEST_CASE("circular low orbit (700 km about a Domus-mass body), 100 orbits, default tolerance")
    {
        // Measured: position error 2.6e-5 of the radius, energy drift 5.4e-8, 142 steps per orbit.
        const Outcome o = runOrbit(kMuDomus, 7.0e5, 0.0, 100, {}, {}, Dopri5Tolerance().rtol);
        CHECK(o.relPosErr < 1e-4);
        CHECK(o.relEnergyDrift < 2e-7);
        CHECK(o.stepsPerOrbit > 100.0);
        CHECK(o.stepsPerOrbit < 200.0);
    }

    TEST_CASE("e = 0.9 orbit (periapsis 500 km), 100 orbits, default tolerance")
    {
        // Measured: position error 5.1e-5 of a, energy drift 1.1e-7, 279 steps per orbit, no rejections (the step grows
        // and shrinks smoothly with the distance).
        const Outcome o = runOrbit(kMuDomus, 5.0e6, 0.9, 100, {}, {}, Dopri5Tolerance().rtol);
        CHECK(o.relPosErr < 2e-4);
        CHECK(o.relEnergyDrift < 5e-7);
        CHECK(o.stepsPerOrbit > 200.0);
        CHECK(o.stepsPerOrbit < 400.0);
    }

    TEST_CASE("tightening the tolerance buys accuracy (circular and e = 0.9)")
    {
        // Measured at rtol 1e-12: 5.7e-7 and 1.0e-6 (about 6.5x per decade of rtol).
        for (double e : {0.0, 0.9}) {
            const double a = e == 0.0 ? 7.0e5 : 5.0e6;
            const Outcome loose = runOrbit(kMuDomus, a, e, 100, {}, {}, 1e-10);
            const Outcome mid = runOrbit(kMuDomus, a, e, 100, {}, {}, 1e-11);
            const Outcome tight = runOrbit(kMuDomus, a, e, 100, {}, {}, 1e-12);
            CAPTURE(e);
            CHECK(mid.relPosErr < loose.relPosErr / 3.0);
            CHECK(tight.relPosErr < mid.relPosErr / 3.0);
            CHECK(tight.relPosErr < 5e-6);
            CHECK(tight.relEnergyDrift < 1e-8);
            CHECK(tight.stepsPerOrbit > loose.stepsPerOrbit * 1.5);
        }
    }

    TEST_CASE("the error scale follows the orbit, not the coordinates: a planet 1.45e10 m from the origin changes nothing")
    {
        // Barycentric coordinates of 1e10 m would make a position-based tolerance 20000x looser for a 700 km orbit.
        for (double e : {0.0, 0.9}) {
            const double a = e == 0.0 ? 7.0e5 : 5.0e6;
            const Outcome near = runOrbit(kMuDomus, a, e, 50, {}, {}, 1e-10);
            const Outcome far = runOrbit(kMuDomus, a, e, 50, {1.45e10, -3.0e9, 0.0}, {}, 1e-10);
            CAPTURE(e);
            CHECK(far.stepsPerOrbit == doctest::Approx(near.stepsPerOrbit).epsilon(0.01));
            CHECK(far.relPosErr < 1e-4);
            CHECK(far.relEnergyDrift < 5e-7);
        }
    }

    TEST_CASE("the orbit of a body that moves at 30 km/s is integrated as accurately (time-dependent body position)")
    {
        // Galilean boost: the shuttle's orbit about the moving body is the same Kepler orbit.
        const Outcome still = runOrbit(kMuDomus, 7.0e5, 0.0, 50, {}, {}, 1e-10);
        const Outcome moving = runOrbit(kMuDomus, 7.0e5, 0.0, 50, {1.0e9, 0.0, 0.0}, {3.0e4, 1.0e4, 0.0}, 1e-10);
        CHECK(moving.relPosErr < 1e-4);
        CHECK(moving.relPosErr < 3.0 * still.relPosErr + 1e-6);
        CHECK(moving.stepsPerOrbit == doctest::Approx(still.stepsPerOrbit).epsilon(0.01));
    }

    TEST_CASE("scale invariance: a 1000x wider orbit with the same period takes the same number of steps")
    {
        // a -> 1000 a, mu -> 1e9 mu keeps the period, and the relative tolerance sees the same orbit shape.
        const Outcome small = runOrbit(kMuDomus, 7.0e5, 0.3, 20, {}, {}, 1e-10);
        const Outcome large = runOrbit(kMuDomus * 1e9, 7.0e8, 0.3, 20, {}, {}, 1e-10);
        CHECK(large.stepsPerOrbit == doctest::Approx(small.stepsPerOrbit).epsilon(0.03));
        CHECK(large.relPosErr < 1e-4);
        CHECK(small.relPosErr < 1e-4);
    }

    TEST_CASE("an interval cut into pieces gives the same orbit as one call (steps end at the interval ends)")
    {
        Bodies b = singleBody(kMuDomus);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        const RelState st = keplerAt(kMuDomus, 7.0e5, 0.2, 0.0, 1.0, 0.0);
        Shuttles one, cut;
        REQUIRE(one.add(shuttleDesc(1, st.r, st.v)));
        REQUIRE(cut.add(shuttleDesc(1, st.r, st.v)));
        const double T = 10.0 * keplerPeriod(kMuDomus, 7.0e5);
        std::vector<Event> ev;
        propagateShuttle(one, 0, 0.0, T, env, ev);
        const int pieces = 137;
        for (int k = 0; k < pieces; ++k) propagateShuttle(cut, 0, T * k / pieces, T * (k + 1) / pieces, env, ev);
        const RelState ref = keplerAt(kMuDomus, 7.0e5, 0.2, 0.0, 1.0, T);
        CHECK(norm(shuttlePos(one, 0) - ref.r) / 7.0e5 < 1e-5);
        CHECK(norm(shuttlePos(cut, 0) - ref.r) / 7.0e5 < 1e-5);
        CHECK(norm(shuttlePos(cut, 0) - shuttlePos(one, 0)) / 7.0e5 < 1e-5);
    }
}
