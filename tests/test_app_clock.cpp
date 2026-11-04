#include <doctest.h>
#include <cmath>
#include <limits>
#include "app/clock.h"

using namespace cosmos::app;

// Sim clock (plan 4.4): t_target += warp * frameDt, frameDt clamped, pause, compensated summation.

TEST_SUITE("app clock")
{
    TEST_CASE("one frame adds warp times frameDt")
    {
        SimClock c(100.0);
        CHECK(c.target() == 100.0);
        const double added = c.advance(1.0 / 64.0, 1000.0);   // exact binary numbers: 15.625 s
        CHECK(added == 15.625);
        CHECK(c.target() == 115.625);
        CHECK(c.lastFrameDt() == 1.0 / 64.0);
        CHECK_FALSE(c.lastFrameClamped());
    }

    TEST_CASE("a stalled frame is clamped to the maximum frame time")
    {
        SimClock c(0.0);
        CHECK(c.maxFrameDt() == 0.25);
        const double added = c.advance(5.0, 10.0);   // a 5 s stall counts as 0.25 s
        CHECK(added == 2.5);
        CHECK(c.lastFrameDt() == 0.25);
        CHECK(c.lastFrameClamped());

        SimClock d(0.0, 0.1);   // custom limit
        CHECK(d.advance(1.0, 1.0) == 0.1);
    }

    TEST_CASE("negative, NaN and infinite inputs add nothing or are bounded")
    {
        SimClock c(5.0);
        CHECK(c.advance(-1.0, 100.0) == 0.0);
        CHECK(c.advance(std::numeric_limits<double>::quiet_NaN(), 100.0) == 0.0);
        CHECK(c.advance(0.01, -100.0) == 0.0);   // negative warp
        CHECK(c.advance(0.01, std::numeric_limits<double>::quiet_NaN()) == 0.0);
        CHECK(c.advance(0.01, std::numeric_limits<double>::infinity()) == 0.0);
        CHECK(c.target() == 5.0);
        // an infinite frame time is clamped like any stall
        CHECK(c.advance(std::numeric_limits<double>::infinity(), 1.0) == 0.25);
    }

    TEST_CASE("pause stops the clock, resume continues from the same time")
    {
        SimClock c(10.0);
        c.advance(0.125, 8.0);   // +1
        c.setPaused(true);
        CHECK(c.paused());
        CHECK(c.advance(0.125, 8.0) == 0.0);
        CHECK(c.advance(0.125, 8.0) == 0.0);
        CHECK(c.target() == 11.0);
        CHECK(c.lastFrameDt() == 0.125);   // the frame time is still tracked while paused
        c.setPaused(false);
        c.advance(0.125, 8.0);
        CHECK(c.target() == 12.0);
    }

    TEST_CASE("a warp change applies from the next advance, never to a frame already counted")
    {
        SimClock c(0.0);
        c.advance(0.25, 4.0);   // +1
        const double before = c.target();
        // the caller changes the warp now; nothing already accounted for moves
        CHECK(c.target() == before);
        c.advance(0.25, 400.0);   // +100
        CHECK(c.target() == 101.0);
    }

    TEST_CASE("holdAt moves the display time back, never forward")
    {
        SimClock c(0.0);
        c.advance(0.25, 100.0);   // 25
        c.holdAt(30.0);           // later than now: ignored
        CHECK(c.target() == 25.0);
        c.holdAt(20.0);
        CHECK(c.target() == 20.0);
        c.advance(0.25, 100.0);
        CHECK(c.target() == 45.0);
        c.reset(1.0e6);
        CHECK(c.target() == 1.0e6);
    }

    TEST_CASE("compensated summation: a million frames at 144 Hz do not drift at a large time")
    {
        // At 1e9 s one ulp is 1.2e-7 s and a frame step is 6.9e-3 s, so every plain addition rounds by up to 6e-8 s and the
        // errors add up. The clock keeps the lost part.
        const double start = 1.0e9;
        const double dt = 1.0 / 144.0;   // the double nearest to it, the same value in every frame
        const long n = 1000000;
        SimClock c(start);
        double naive = start;
        for (long i = 0; i < n; ++i) {
            c.advance(dt, 1.0);
            naive += dt;
        }
        const long double exact = static_cast<long double>(start) + static_cast<long double>(n) * static_cast<long double>(dt);
        const double clockErr = static_cast<double>(std::fabs(static_cast<long double>(c.target()) - exact));
        const double naiveErr = static_cast<double>(std::fabs(static_cast<long double>(naive) - exact));
        // The clock is within rounding of the final value (one ulp of 1e9 is 1.19e-7; the long double reference adds ~1e-12)
        CHECK(clockErr <= 1.2e-7);
        // The plain sum is far off, so this test would fail without the compensation
        CHECK(naiveErr > 1.0e-4);
        CHECK(clockErr * 1000.0 < naiveErr);
    }

    TEST_CASE("a small step is not lost against a huge time")
    {
        SimClock c(1.0e16);   // ulp of 1e16 is 2: a step of 0.25 s vanishes in plain addition
        for (int i = 0; i < 1600; ++i) c.advance(0.25, 1.0);   // +400 s in total
        CHECK(c.target() == 1.0e16 + 400.0);
    }
}
