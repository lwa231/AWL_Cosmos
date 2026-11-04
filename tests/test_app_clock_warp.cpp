#include <doctest.h>
#include <cmath>
#include <limits>
#include "app/warp.h"

using namespace cosmos::app;

// Warp ladder, slider value and the memory behind "Resume warp" (plan 8 items 5-6, 10.3).

TEST_SUITE("app warp")
{
    TEST_CASE("the ladder is 1, 10, ... 1e6")
    {
        REQUIRE(kWarpLadder.size() == 7);
        const double expect[7] = {1.0, 10.0, 100.0, 1.0e3, 1.0e4, 1.0e5, 1.0e6};
        for (std::size_t i = 0; i < 7; ++i) CHECK(kWarpLadder[i] == expect[i]);
        CHECK(kWarpMin == 0.0);
        CHECK(kWarpMax == 1.0e6);
    }

    TEST_CASE("stepUp and stepDown walk the rungs and stop at the ends")
    {
        Warp w(1.0);
        CHECK(w.value() == 1.0);
        CHECK_FALSE(w.stepDown());   // already at the bottom
        CHECK(w.value() == 1.0);
        for (int i = 1; i < 7; ++i) {
            CHECK(w.stepUp());
            CHECK(w.value() == kWarpLadder[static_cast<std::size_t>(i)]);
            CHECK(w.ladderIndex() == i);
        }
        CHECK_FALSE(w.stepUp());   // top
        CHECK(w.value() == 1.0e6);
        for (int i = 5; i >= 0; --i) {
            CHECK(w.stepDown());
            CHECK(w.value() == kWarpLadder[static_cast<std::size_t>(i)]);
        }
    }

    TEST_CASE("from a value between rungs, stepping goes to the neighbour rung")
    {
        Warp w(250.0);
        CHECK(w.ladderIndex() == -1);
        CHECK(w.stepUp());
        CHECK(w.value() == 1000.0);
        w.set(250.0);
        CHECK(w.stepDown());
        CHECK(w.value() == 100.0);
        // slow motion: up goes to 1, down stays
        w.set(0.3);
        CHECK_FALSE(w.stepDown());
        CHECK(w.value() == 0.3);
        CHECK(w.stepUp());
        CHECK(w.value() == 1.0);
        // a value within 1e-9 of a rung counts as on it
        w.set(100.0 * (1.0 + 1.0e-12));
        CHECK(w.ladderIndex() == 2);
        CHECK(w.stepUp());
        CHECK(w.value() == 1000.0);
    }

    TEST_CASE("set clamps to [0, 1e6] and ignores NaN")
    {
        Warp w(10.0);
        w.set(-5.0);
        CHECK(w.value() == 0.0);
        w.set(2.0e6);
        CHECK(w.value() == 1.0e6);
        w.set(std::numeric_limits<double>::infinity());
        CHECK(w.value() == 1.0e6);
        w.set(123.0);
        w.set(std::numeric_limits<double>::quiet_NaN());
        CHECK(w.value() == 123.0);
        CHECK(Warp(-3.0).value() == 0.0);
        CHECK(Warp(1.0e9).value() == 1.0e6);
    }

    TEST_CASE("slow-down goes one level at a time and remembers where it started")
    {
        Warp w(1.0e5);
        CHECK_FALSE(w.canResume());
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 1.0e4);
        CHECK(w.canResume());
        CHECK(w.resumeValue() == 1.0e5);
        CHECK(w.slowDownOneLevel());
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 100.0);
        CHECK(w.resumeValue() == 1.0e5);   // still the first value, not 1e4
        CHECK(w.slowDownOneLevel());
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 1.0);
        CHECK_FALSE(w.slowDownOneLevel());   // nothing below 1x
        CHECK(w.value() == 1.0);
        CHECK(w.canResume());
        CHECK(w.resume());
        CHECK(w.value() == 1.0e5);
        CHECK_FALSE(w.canResume());
        CHECK_FALSE(w.resume());   // nothing left to restore
    }

    TEST_CASE("slow-down from a slider value between rungs restores that exact value")
    {
        Warp w(34567.0);
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 1.0e4);
        CHECK(w.resume());
        CHECK(w.value() == 34567.0);
    }

    TEST_CASE("no slow-down at 1x or below, and nothing to resume")
    {
        Warp w(1.0);
        CHECK_FALSE(w.slowDownOneLevel());
        CHECK_FALSE(w.canResume());
        w.set(0.5);
        CHECK_FALSE(w.slowDownOneLevel());
        CHECK_FALSE(w.canResume());
        CHECK(w.value() == 0.5);
    }

    TEST_CASE("manual changes keep the memory; clearResume forgets it; a new slow-down after a speed-up restarts it")
    {
        Warp w(1.0e6);
        w.slowDownOneLevel();   // 1e5, remembers 1e6
        w.stepDown();           // the user goes lower by hand: 1e4
        CHECK(w.canResume());
        CHECK(w.resumeValue() == 1.0e6);
        w.set(1.0e6);           // the user is back at the remembered value: nothing to resume
        CHECK_FALSE(w.canResume());
        w.set(1.0e3);
        w.slowDownOneLevel();   // a new slow-down starts from 1e3, replacing the old memory
        CHECK(w.value() == 100.0);
        CHECK(w.resumeValue() == 1.0e3);
        w.clearResume();
        CHECK_FALSE(w.canResume());
        CHECK(w.value() == 100.0);
    }

    TEST_CASE("slider mapping is logarithmic over 1..1e6 and round-trips")
    {
        CHECK(Warp::sliderFromWarp(1.0) == 0.0);
        CHECK(Warp::sliderFromWarp(0.2) == 0.0);
        CHECK(Warp::sliderFromWarp(1.0e6) == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(Warp::sliderFromWarp(1.0e3) == doctest::Approx(0.5).epsilon(1e-12));
        CHECK(Warp::sliderFromWarp(1.0e9) == 1.0);   // clamped
        CHECK(Warp::warpFromSlider(0.0) == 1.0);
        CHECK(Warp::warpFromSlider(1.0) == doctest::Approx(1.0e6).epsilon(1e-12));
        CHECK(Warp::warpFromSlider(-1.0) == 1.0);
        CHECK(Warp::warpFromSlider(2.0) == doctest::Approx(1.0e6).epsilon(1e-12));
        CHECK(Warp::warpFromSlider(std::numeric_limits<double>::quiet_NaN()) == 1.0);
        for (double s = 0.05; s <= 1.0; s += 0.05)
            CHECK(std::fabs(Warp::sliderFromWarp(Warp::warpFromSlider(s)) - s) < 1.0e-12);
        // every ladder rung sits at k/6 of the slider
        for (std::size_t k = 1; k < 7; ++k)
            CHECK(std::fabs(Warp::sliderFromWarp(kWarpLadder[k]) - static_cast<double>(k) / 6.0) < 1.0e-12);
    }
}
