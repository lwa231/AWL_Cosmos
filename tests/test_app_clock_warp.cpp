#include <doctest.h>
#include <cmath>
#include <limits>
#include "app/warp.h"

using namespace cosmos::app;

// Warp ladder (the V1.1 stops), slider value and the memory behind "Resume warp" (plan 8 items 5-6, 10.3; V1.1 step 2).

TEST_SUITE("app warp")
{
    TEST_CASE("the ladder is the V1.1 stops: 1 s/s .. 10 d/s")
    {
        REQUIRE(kWarpLadder.size() == 18);
        const double expect[18] = {1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600, 7200, 21600, 43200, 86400, 172800, 432000, 864000};
        for (std::size_t i = 0; i < 18; ++i) CHECK(kWarpLadder[i] == expect[i]);
        CHECK(kWarpTopStop == kWarpLadder.back());
        CHECK(kWarpMin == 0.0);
        CHECK(kWarpMax == 1.0e6);   // the typed / scripted range stays; the UI ends at the last stop
    }

    TEST_CASE("stepUp and stepDown walk the stops and stop at the ends")
    {
        Warp w(1.0);
        CHECK(w.value() == 1.0);
        CHECK_FALSE(w.stepDown());   // already at the bottom
        CHECK(w.value() == 1.0);
        const int n = static_cast<int>(kWarpLadder.size());
        for (int i = 1; i < n; ++i) {
            CHECK(w.stepUp());
            CHECK(w.value() == kWarpLadder[static_cast<std::size_t>(i)]);
            CHECK(w.ladderIndex() == i);
        }
        CHECK_FALSE(w.stepUp());   // top
        CHECK(w.value() == 864000.0);
        for (int i = n - 2; i >= 0; --i) {
            CHECK(w.stepDown());
            CHECK(w.value() == kWarpLadder[static_cast<std::size_t>(i)]);
        }
    }

    TEST_CASE("from a value between stops, stepping goes to the neighbour stop")
    {
        Warp w(250.0);
        CHECK(w.ladderIndex() == -1);
        CHECK(w.stepUp());
        CHECK(w.value() == 300.0);
        w.set(250.0);
        CHECK(w.stepDown());
        CHECK(w.value() == 120.0);
        // above the last stop (a scripted 1e6): down goes to the last stop, up changes nothing
        w.set(1.0e6);
        CHECK_FALSE(w.stepUp());
        CHECK(w.stepDown());
        CHECK(w.value() == 864000.0);
        // slow motion: up goes to 1, down stays
        w.set(0.3);
        CHECK_FALSE(w.stepDown());
        CHECK(w.value() == 0.3);
        CHECK(w.stepUp());
        CHECK(w.value() == 1.0);
        // a value within 1e-9 of a stop counts as on it
        w.set(300.0 * (1.0 + 1.0e-12));
        CHECK(w.ladderIndex() == 7);
        CHECK(w.stepUp());
        CHECK(w.value() == 600.0);
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
        Warp w(86400.0);
        CHECK_FALSE(w.canResume());
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 43200.0);
        CHECK(w.canResume());
        CHECK(w.resumeValue() == 86400.0);
        CHECK(w.slowDownOneLevel());
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 7200.0);
        CHECK(w.resumeValue() == 86400.0);   // still the first value, not 43200
        for (int i = 0; i < 11; ++i) CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 1.0);
        CHECK_FALSE(w.slowDownOneLevel());   // nothing below 1x
        CHECK(w.value() == 1.0);
        CHECK(w.canResume());
        CHECK(w.resume());
        CHECK(w.value() == 86400.0);
        CHECK_FALSE(w.canResume());
        CHECK_FALSE(w.resume());   // nothing left to restore
    }

    TEST_CASE("slow-down from a slider value between stops restores that exact value")
    {
        Warp w(34567.0);
        CHECK(w.slowDownOneLevel());
        CHECK(w.value() == 21600.0);
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
        Warp w(864000.0);
        w.slowDownOneLevel();   // 432000, remembers 864000
        w.stepDown();           // the user goes lower by hand: 172800
        CHECK(w.canResume());
        CHECK(w.resumeValue() == 864000.0);
        w.set(864000.0);        // the user is back at the remembered value: nothing to resume
        CHECK_FALSE(w.canResume());
        w.set(600.0);
        w.slowDownOneLevel();   // a new slow-down starts from 600, replacing the old memory
        CHECK(w.value() == 300.0);
        CHECK(w.resumeValue() == 600.0);
        w.clearResume();
        CHECK_FALSE(w.canResume());
        CHECK(w.value() == 300.0);
    }

    TEST_CASE("slider mapping is logarithmic over 1..864000 and round-trips")
    {
        CHECK(Warp::sliderFromWarp(1.0) == 0.0);
        CHECK(Warp::sliderFromWarp(0.2) == 0.0);
        CHECK(Warp::sliderFromWarp(864000.0) == 1.0);
        CHECK(Warp::sliderFromWarp(1.0e6) == 1.0);   // clamped
        CHECK(Warp::sliderFromWarp(std::sqrt(864000.0)) == doctest::Approx(0.5).epsilon(1e-12));
        CHECK(Warp::warpFromSlider(0.0) == 1.0);
        CHECK(Warp::warpFromSlider(1.0) == 864000.0);
        CHECK(Warp::warpFromSlider(-1.0) == 1.0);
        CHECK(Warp::warpFromSlider(2.0) == 864000.0);
        CHECK(Warp::warpFromSlider(std::numeric_limits<double>::quiet_NaN()) == 1.0);
        for (double s = 0.05; s <= 1.0; s += 0.05)
            CHECK(std::fabs(Warp::sliderFromWarp(Warp::warpFromSlider(s)) - s) < 1.0e-12);
    }

    TEST_CASE("nearestStop snaps on the log scale; a drag snaps unless smooth")
    {
        for (double stop : kWarpLadder) {
            CHECK(nearestStop(stop) == stop);
            CHECK(nearestStop(stop * 1.05) == stop);
            CHECK(nearestStop(stop / 1.05) == stop);
        }
        CHECK(nearestStop(0.0) == 1.0);
        CHECK(nearestStop(0.3) == 1.0);
        CHECK(nearestStop(std::numeric_limits<double>::quiet_NaN()) == 1.0);
        CHECK(nearestStop(5.0e6) == 864000.0);
        // the geometric midpoint of 10 and 30 is 17.3
        CHECK(nearestStop(17.0) == 10.0);
        CHECK(nearestStop(17.5) == 30.0);
        CHECK(nearestStop(3000.0) == 3600.0);
        CHECK(nearestStop(2000.0) == 1800.0);
        // every slider position lands on a stop; Ctrl gives the smooth value
        for (double s = 0.0; s <= 1.0; s += 0.01) {
            const double w = Warp::warpFromSliderDrag(s, false);
            bool onStop = false;
            for (double stop : kWarpLadder) onStop = onStop || w == stop;
            CHECK(onStop);
            CHECK(Warp::warpFromSliderDrag(s, true) == Warp::warpFromSlider(s));
        }
        CHECK(Warp::warpFromSliderDrag(1.0, false) == 864000.0);
        CHECK(Warp::warpFromSliderDrag(0.0, false) == 1.0);
    }

    TEST_CASE("formatWarp reads like the stops")
    {
        CHECK(formatWarp(1.0) == "1 s/s");
        CHECK(formatWarp(30.0) == "30 s/s");
        CHECK(formatWarp(60.0) == "1 min/s");
        CHECK(formatWarp(1800.0) == "30 min/s");
        CHECK(formatWarp(7200.0) == "2 h/s");
        CHECK(formatWarp(21600.0) == "6 h/s");
        CHECK(formatWarp(432000.0) == "5 d/s");
        CHECK(formatWarp(864000.0) == "10 d/s");
        CHECK(formatWarp(0.0) == "0 s/s");
        CHECK(formatWarp(0.5) == "0.5 s/s");
        CHECK(formatWarp(4932.0) == "1.37 h/s");
        CHECK(formatWarp(90.0) == "1.5 min/s");
        CHECK(formatWarp(1.0e6) == "11.6 d/s");
        CHECK(formatWarp(std::numeric_limits<double>::infinity()) == "-- /s");
    }
}
