#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// Hull heading and spin of the drawn shuttles (plan 6.6). The controller changes the torque inside a body step, so the
// heading cannot be extrapolated from the end of the step; it is a Hermite of the recorded per-substep samples.

namespace {
constexpr double kPi = 3.14159265358979323846;
double wrapPi(double a) { return std::remainder(a, 2.0 * kPi); }
}

TEST_SUITE("app driver heading")
{
    TEST_CASE("the drawn heading and spin match the shuttle at step ends and join across body steps")
    {
        ResolvedScene res = lucernaWithShuttles(3);   // Coast, Prograde, Wander
        SimDriver d(res, SceneSettings{});
        double worstEndHeading = 0.0, worstEndSpin = 0.0, worstJoinHeading = 0.0, worstJoinSpin = 0.0, maxSpin = 0.0;
        DisplayState prevEnd;
        for (int step = 0; step < 80; ++step) {
            d.advanceTo(justAfter(d.simTime()));
            const Shuttles& S = d.sim().shuttles();
            const DisplayState atEnd = d.stateAt(d.sim().bodyStepEnd());
            const DisplayState atStart = d.stateAt(d.sim().bodyStepStart());
            for (std::size_t i = 0; i < S.size(); ++i) {
                if (!S.alive[i]) continue;
                const DisplayShuttle* e = findShuttle(atEnd, S.ids[i]);
                REQUIRE(e != nullptr);
                worstEndHeading = std::max(worstEndHeading, std::fabs(wrapPi(e->heading - S.heading[i])));
                worstEndSpin = std::max(worstEndSpin, std::fabs(e->spin - S.spin[i]));
                maxSpin = std::max(maxSpin, std::fabs(S.spin[i]));
                if (step > 0) {
                    const DisplayShuttle* p = findShuttle(prevEnd, S.ids[i]);
                    const DisplayShuttle* s = findShuttle(atStart, S.ids[i]);
                    REQUIRE(p != nullptr);
                    REQUIRE(s != nullptr);
                    worstJoinHeading = std::max(worstJoinHeading, std::fabs(wrapPi(s->heading - p->heading)));
                    worstJoinSpin = std::max(worstJoinSpin, std::fabs(s->spin - p->spin));
                }
            }
            prevEnd = atEnd;
        }
        MESSAGE("end: heading " << worstEndHeading << " rad, spin " << worstEndSpin << " rad/s; join: heading " << worstJoinHeading
                                << " rad, spin " << worstJoinSpin << " rad/s; max |spin| " << maxSpin);
        REQUIRE(maxSpin > 1.0e-3);   // the scenario really turns
        // The Hermite is exact at its samples: the only difference is the rounding of the unwrapped heading, summed over
        // all the substeps of a body step (measured 1.8e-10 rad, headings reach many turns); 1e-9 rad is invisible
        CHECK(worstEndHeading < 1.0e-9);
        CHECK(worstEndSpin < 1.0e-12);
        CHECK(worstJoinHeading < 1.0e-9);
        CHECK(worstJoinSpin < 1.0e-12);
    }

    TEST_CASE("the heading moves smoothly inside and across body steps, at the speed of the spin")
    {
        ResolvedScene res = lucernaWithShuttles(3);
        SimDriver d(res, SceneSettings{});
        const double dt = 0.05;   // s between the sampled display times (half a controller tick, 0.1 s)
        double worstExcess = 0.0, worstSpinStep = 0.0, maxSpin = 0.0, alphaMax = 0.0;
        const Shuttles& S = d.sim().shuttles();
        for (std::size_t i = 0; i < S.size(); ++i) alphaMax = std::max(alphaMax, S.maxTorque[i] / S.inertia[i]);
        DisplayState prev = d.stateAt(0.0);
        long samples = 0;
        for (long k = 1; k <= 8000; ++k) {   // 400 s: many body steps
            const double t = static_cast<double>(k) * dt;
            d.advanceTo(t);
            const DisplayState cur = d.stateAt(t);
            for (const DisplayShuttle& s : cur.shuttles) {
                const DisplayShuttle* p = findShuttle(prev, s.id);
                REQUIRE(p != nullptr);
                // The heading is quadratic between controller ticks, so over dt it changes by the mean of the two spins
                // times dt, except when the torque switches inside the interval, where the spin has a kink and the
                // trapezoid is off by at most alpha dt^2 / 4. Whole turns are not compared (the value is modulo 2 pi).
                const double change = wrapPi(s.heading - p->heading);
                const double expect = 0.5 * (s.spin + p->spin) * dt;
                worstExcess = std::max(worstExcess, std::fabs(wrapPi(change - expect)));
                worstSpinStep = std::max(worstSpinStep, std::fabs(s.spin - p->spin));
                maxSpin = std::max(maxSpin, std::fabs(s.spin));
                ++samples;
            }
            prev = cur;
        }
        MESSAGE(samples << " samples; worst |change - mean spin * dt| " << worstExcess << " rad (bound " << 0.25 * alphaMax * dt * dt
                        << "), worst spin step " << worstSpinStep << " rad/s (bound " << alphaMax * dt << "), max |spin| " << maxSpin);
        REQUIRE(maxSpin > 1.0e-3);
        CHECK(worstExcess <= 0.25 * alphaMax * dt * dt * (1.0 + 1.0e-6) + 1.0e-9);
        CHECK(worstSpinStep <= alphaMax * dt * (1.0 + 1.0e-6) + 1.0e-12);
    }
}
