#include <doctest.h>
#include <limits>
#include <string>
#include "app/format.h"

using namespace cosmos::app;

// Readout formatting (plan 10.2, 10.3): significant figures, time rate, date and clock, distance, speed.

TEST_SUITE("app format")
{
    TEST_CASE("formatSig keeps the requested significant figures without exponent form")
    {
        CHECK(formatSig(14.93) == "14.9");
        CHECK(formatSig(1.0) == "1.00");
        CHECK(formatSig(0.01234) == "0.0123");
        CHECK(formatSig(1.0e-5) == "0.0000100");
        CHECK(formatSig(123456.0) == "123456");   // more integer digits than figures: all are kept
        CHECK(formatSig(-2.5) == "-2.50");
        CHECK(formatSig(0.0) == "0.00");
        CHECK(formatSig(-0.0) == "0.00");
        CHECK(formatSig(0.0456, 1) == "0.05");
        CHECK(formatSig(0.0, 1) == "0");
        CHECK(formatSig(1234.5678, 6) == "1234.57");
        CHECK(formatSig(1234.5678, 20) == "1234.56780");   // sig is clamped to 9
    }

    TEST_CASE("formatSig handles rounding that carries into a new digit")
    {
        CHECK(formatSig(9.996) == "10.0");
        CHECK(formatSig(99.96) == "100");
        CHECK(formatSig(999.7) == "1000");
        CHECK(formatSig(0.09996) == "0.100");
        CHECK(formatSig(9.994) == "9.99");
    }

    TEST_CASE("formatSig non-finite values")
    {
        CHECK(formatSig(std::numeric_limits<double>::quiet_NaN()) == "nan");
        CHECK(formatSig(std::numeric_limits<double>::infinity()) == "inf");
        CHECK(formatSig(-std::numeric_limits<double>::infinity()) == "-inf");
    }

    TEST_CASE("time rate in automatic units, 3 significant figures")
    {
        CHECK(formatTimeRate(1.0) == "1.00 s/sec");
        CHECK(formatTimeRate(10.0) == "10.0 s/sec");
        CHECK(formatTimeRate(100.0) == "1.67 min/sec");
        CHECK(formatTimeRate(1000.0) == "16.7 min/sec");
        CHECK(formatTimeRate(1.0e4) == "2.78 h/sec");
        CHECK(formatTimeRate(1.0e5) == "1.16 days/sec");
        CHECK(formatTimeRate(1.0e6) == "11.6 days/sec");
        CHECK(formatTimeRate(1285000.0) == "14.9 days/sec");   // the example of plan 10.3
        CHECK(formatTimeRate(1.0e7) == "116 days/sec");
        CHECK(formatTimeRate(3.1557600e7) == "1.00 years/sec");   // one Julian year
        CHECK(formatTimeRate(3.1557600e8) == "10.0 years/sec");
        CHECK(formatTimeRate(0.5) == "0.500 s/sec");
        CHECK(formatTimeRate(0.0) == "0.00 s/sec");
        CHECK(formatTimeRate(-100.0) == "-1.67 min/sec");
    }

    TEST_CASE("time rate switches unit on the displayed value, never showing 60.0 min or 24.0 h")
    {
        CHECK(formatTimeRate(59.0) == "59.0 s/sec");
        CHECK(formatTimeRate(60.0) == "1.00 min/sec");
        CHECK(formatTimeRate(59.96) == "60.0 s/sec");   // in minutes it would read 0.999
        CHECK(formatTimeRate(3590.0) == "59.8 min/sec");
        CHECK(formatTimeRate(3599.0) == "1.00 h/sec");
        CHECK(formatTimeRate(3600.0) == "1.00 h/sec");
        CHECK(formatTimeRate(86399.0) == "1.00 days/sec");
        CHECK(formatTimeRate(86400.0) == "1.00 days/sec");
        CHECK(formatTimeRate(3.15e7) == "365 days/sec");
        CHECK(formatTimeRate(std::numeric_limits<double>::infinity()) == "-- /sec");
        CHECK(formatTimeRate(std::numeric_limits<double>::quiet_NaN()) == "-- /sec");
    }

    TEST_CASE("simulated date and clock")
    {
        CHECK(formatSimDate(0.0) == "Day 0  00:00:00");
        CHECK(formatSimDate(1.0) == "Day 0  00:00:01");
        CHECK(formatSimDate(3661.9) == "Day 0  01:01:01");   // seconds are truncated
        CHECK(formatSimDate(86399.999) == "Day 0  23:59:59");
        CHECK(formatSimDate(86400.0) == "Day 1  00:00:00");
        CHECK(formatSimDate(123.0 * 86400.0 + 4.0 * 3600.0 + 5.0 * 60.0 + 6.0) == "Day 123  04:05:06");
        CHECK(formatSimDate(1.0e9) == "Day 11574  01:46:40");
        CHECK(formatSimDate(-1.0) == "-Day 0  00:00:01");
        CHECK(formatSimDate(-90000.0) == "-Day 1  01:00:00");
        CHECK(formatSimDate(-0.5) == "Day 0  00:00:00");
        CHECK(formatSimDate(std::numeric_limits<double>::quiet_NaN()) == "Day --");
        CHECK(formatSimDate(std::numeric_limits<double>::infinity()) == "Day --");
        CHECK(formatSimDate(1.0e30).substr(0, 4) == "Day ");   // huge values stay safe
    }

    TEST_CASE("distance in m, km, Mm, Gm, AU")
    {
        CHECK(formatDistance(0.0) == "0.00 m");
        CHECK(formatDistance(1.0e-3) == "0.00100 m");
        CHECK(formatDistance(845.0) == "845 m");
        CHECK(formatDistance(1000.0) == "1.00 km");
        CHECK(formatDistance(6.2e5) == "620 km");
        CHECK(formatDistance(1.0e6) == "1.00 Mm");
        CHECK(formatDistance(1.36e10) == "13.6 Gm");
        CHECK(formatDistance(5.0e10) == "50.0 Gm");
        CHECK(formatDistance(1.0e11) == "0.668 AU");
        CHECK(formatDistance(1.495978707e11) == "1.00 AU");
        CHECK(formatDistance(4.5e12) == "30.1 AU");
        CHECK(formatDistance(-2500.0) == "-2.50 km");
        CHECK(formatDistance(std::numeric_limits<double>::infinity()) == "inf m");
    }

    TEST_CASE("distance unit changes on the displayed value")
    {
        CHECK(formatDistance(999.4) == "999 m");
        CHECK(formatDistance(999.6) == "1.00 km");
        CHECK(formatDistance(999.4e3) == "999 km");
        CHECK(formatDistance(999.6e3) == "1.00 Mm");
        CHECK(formatDistance(9.996e10) == "0.668 AU");   // 99.96 Gm: reads the same as the AU threshold
    }

    TEST_CASE("speed in m/s and km/s")
    {
        CHECK(formatSpeed(0.0) == "0.00 m/s");
        CHECK(formatSpeed(850.0) == "850 m/s");
        CHECK(formatSpeed(999.4) == "999 m/s");
        CHECK(formatSpeed(999.7) == "1.00 km/s");
        CHECK(formatSpeed(2170.0) == "2.17 km/s");
        CHECK(formatSpeed(-1200.0) == "-1.20 km/s");
        CHECK(formatSpeed(2.998e8) == "299800 km/s");
    }
}
