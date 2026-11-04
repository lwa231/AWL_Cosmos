#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// SimDriver (plan 4.4, 6.6): the render state at t_target is exact at body-step boundaries, continuous inside a step, and
// the same whatever sequence of frames led to the same t_target.

TEST_SUITE("app driver")
{
    TEST_CASE("the first state is the scene at the start time, not interpolated")
    {
        ResolvedScene res = lucernaWithShuttles(3);
        SimDriver d(res, SceneSettings{});
        const DisplayState& s = d.state();
        CHECK(s.t == 0.0);
        REQUIRE(s.bodies.size() == res.bodies.size());
        REQUIRE(s.shuttles.size() == 3);
        for (std::size_t i = 0; i < s.bodies.size(); ++i) {
            CHECK(s.bodies[i].id == res.bodies.ids[i]);
            CHECK(sameBits(s.bodies[i].r, sim_test::bodyPos(d.sim().bodies(), i)));
            CHECK_FALSE(s.bodies[i].interpolated);
            CHECK_FALSE(s.bodies[i].absorbed);
        }
        for (const DisplayShuttle& sh : s.shuttles) {
            CHECK_FALSE(sh.interpolated);
            CHECK_FALSE(sh.dying);
        }
        CHECK(d.simTime() == 0.0);
        CHECK(d.targetTime() == 0.0);
    }

    TEST_CASE("the start time of the options is the first display time")
    {
        ResolvedScene res = lucernaWithShuttles(0);
        SimulationOptions opt;
        opt.startTime = 5000.0;
        SimDriver d(res, SceneSettings{}, opt);
        CHECK(d.targetTime() == 5000.0);
        CHECK(d.simTime() == 5000.0);
        CHECK(d.state().t == 5000.0);
        d.advance(0.125);   // warp 1: 0.125 s
        CHECK(d.targetTime() == 5000.125);
    }

    TEST_CASE("exact at body-step boundaries and continuous across them")
    {
        ResolvedScene res = lucernaWithShuttles(3);
        SimDriver d(res, SceneSettings{});
        DisplayState prevEnd;
        double worstBodyEnd = 0.0, worstBodyJoin = 0.0, worstVelEnd = 0.0;
        int checkedJoins = 0;
        for (int step = 0; step < 60; ++step) {
            d.advanceTo(justAfter(d.simTime()));   // exactly one more body step
            REQUIRE(d.sim().hasBodyStep());
            const double t0 = d.sim().bodyStepStart(), t1 = d.sim().bodyStepEnd();
            REQUIRE(t1 > t0);
            CHECK(t1 == d.simTime());

            const DisplayState atEnd = d.stateAt(t1);
            const DisplayState atStart = d.stateAt(t0);
            // End of the step: the store state of the simulation
            const Bodies& B = d.sim().bodies();
            for (std::size_t i = 0; i < B.size(); ++i) {
                const DisplayBody* b = findBody(atEnd, B.ids[i]);
                REQUIRE(b != nullptr);
                worstBodyEnd = std::max(worstBodyEnd, norm(b->r - sim_test::bodyPos(B, i)));
                worstVelEnd = std::max(worstVelEnd, norm(b->v - sim_test::bodyVel(B, i)));
            }
            const Shuttles& S = d.sim().shuttles();
            for (std::size_t i = 0; i < S.size(); ++i) {
                if (!S.alive[i]) continue;
                const DisplayShuttle* sh = findShuttle(atEnd, S.ids[i]);
                REQUIRE(sh != nullptr);
                // Cubic Hermite is exact at its samples: bit for bit, position and velocity
                CHECK(sameBits(sh->r, sim_test::shuttlePos(S, i)));
                CHECK(sameBits(sh->v, sim_test::shuttleVel(S, i)));
            }
            // Start of this step equals the end of the previous one
            if (step > 0) {
                ++checkedJoins;
                for (const DisplayBody& b : atStart.bodies) {
                    const DisplayBody* p = findBody(prevEnd, b.id);
                    REQUIRE(p != nullptr);
                    worstBodyJoin = std::max(worstBodyJoin, norm(b.r - p->r));
                }
                for (const DisplayShuttle& sh : atStart.shuttles) {
                    const DisplayShuttle* p = findShuttle(prevEnd, sh.id);
                    REQUIRE(p != nullptr);
                    CHECK(sameBits(sh.r, p->r));
                    CHECK(sameBits(sh.v, p->v));
                }
            }
            prevEnd = atEnd;
        }
        CHECK(checkedJoins == 59);
        MESSAGE("worst body error at step end " << worstBodyEnd << " m, velocity " << worstVelEnd << " m/s, at joins " << worstBodyJoin << " m");
        // IAS15 dense output evaluated at s = 1 against the stored end state: measured 1.6e-5 m, one ulp of a position near 1e11 m
        CHECK(worstBodyEnd < 1.0e-4);
        CHECK(worstVelEnd < 1.0e-9);
        CHECK(worstBodyJoin < 1.0e-4);
    }

    TEST_CASE("inside a body step the motion is continuous and the velocity is its derivative")
    {
        ResolvedScene res = lucernaWithShuttles(3);
        SimDriver d(res, SceneSettings{});
        for (int i = 0; i < 20; ++i) d.advanceTo(justAfter(d.simTime()));
        const double t0 = d.sim().bodyStepStart(), t1 = d.sim().bodyStepEnd();
        const double h = t1 - t0;
        REQUIRE(h > 10.0);   // a long step, so that sampling inside it is meaningful

        const int n = 400;
        double worstJump = 0.0, worstDeriv = 0.0, worstShuttleDeriv = 0.0;
        DisplayState prev = d.stateAt(t0);
        for (int k = 1; k <= n; ++k) {
            const double t = t0 + h * k / n;
            const DisplayState cur = d.stateAt(t);
            const double dt = t - (t0 + h * (k - 1) / n);
            for (const DisplayBody& b : cur.bodies) {
                const DisplayBody* p = findBody(prev, b.id);
                REQUIRE(p != nullptr);
                // The displacement cannot be more than the faster of the two end speeds times dt, plus the acceleration term
                const double vmax = std::max(norm(b.v), norm(p->v));
                worstJump = std::max(worstJump, norm(b.r - p->r) / (vmax * dt + 1.0e-9));
            }
            prev = cur;
        }
        // central-difference derivative of the position against the velocity of the interpolant, mid-step
        const double eps = 0.1;   // s: truncation eps^2/6 |r'''| stays below 1e-4 m/s for these orbits, rounding (ulp of 1e11 m is 1.5e-5 m) / 2 eps stays below 1e-4
        for (const double s : {0.2, 0.5, 0.8}) {
            const double t = t0 + s * h;
            const DisplayState a = d.stateAt(t - eps), m = d.stateAt(t), b = d.stateAt(t + eps);
            for (const DisplayBody& body : m.bodies) {
                const Vec3 deriv = (findBody(b, body.id)->r - findBody(a, body.id)->r) / (2.0 * eps);
                worstDeriv = std::max(worstDeriv, norm(deriv - body.v));
            }
            for (const DisplayShuttle& sh : m.shuttles) {
                const Vec3 deriv = (findShuttle(b, sh.id)->r - findShuttle(a, sh.id)->r) / (2.0 * eps);
                worstShuttleDeriv = std::max(worstShuttleDeriv, norm(deriv - sh.v));
            }
        }
        MESSAGE("step length " << h << " s, worst jump ratio " << worstJump << ", body derivative error " << worstDeriv
                               << " m/s, shuttle derivative error " << worstShuttleDeriv << " m/s");
        CHECK(worstJump <= 1.05);
        CHECK(worstDeriv < 5.0e-4);
        CHECK(worstShuttleDeriv < 5.0e-4);
    }
}
