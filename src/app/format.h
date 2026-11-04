#pragma once
// Readout formatting for the bottom bar and panels (plan sections 10.2, 10.3). Pure functions, no GL, no locale
// dependence beyond the C locale the program starts in (numbers use '.' and no thousands separators).
#include <string>

namespace cosmos {
namespace app {

// value with `sig` significant figures, never in exponent form: 14.93 -> "14.9", 0.01234 -> "0.0123", 999.7 -> "1000"
// (a value with more integer digits than `sig` keeps all of them). Rounding that carries into a new digit (9.996 -> "10.0")
// is handled. NaN -> "nan", infinities -> "inf" / "-inf". sig is clamped to 1..9.
std::string formatSig(double value, int sig = 3);

// Time rate (sim seconds per real second) in automatic units, 3 significant figures, as "<value> <unit>/sec":
//   s (< 1 min), min (< 1 h), h (< 1 day), days (< 1 year), years (Julian year, 365.25 days).
// The unit changes when the ROUNDED value reaches the next unit ("60.0 s" never shows, it is "1.00 min"). Examples:
// 1 -> "1.00 s/sec", 1285000 -> "14.9 days/sec", 1e6 -> "11.6 days/sec", 0 -> "0.00 s/sec", 0.5 -> "0.500 s/sec".
// Negative rates get a leading '-'; non-finite input gives "-- /sec".
std::string formatTimeRate(double simSecondsPerSecond);

// Simulated date and clock from t_sim (seconds since the scene epoch): "Day 123  04:05:06". The day count is whole
// elapsed days (86400 s), counted from 0 at the epoch; seconds are truncated, not rounded, so the clock never shows a
// moment that has not happened yet. A negative time is the same format of its magnitude with a leading '-': t = -1 s is
// "-Day 0  00:00:01". Non-finite input gives "Day --".
std::string formatSimDate(double tSeconds);

// Length in metres in the largest fitting unit of m, km, Mm, Gm, AU with 3 significant figures:
// "845 m", "620 km", "13.6 Gm", "0.668 AU" (AU from 1e11 m on). Negative values get a '-'.
std::string formatDistance(double metres);

// Speed in m/s below 1 km/s, km/s from there on, 3 significant figures: "850 m/s", "2.17 km/s".
std::string formatSpeed(double metresPerSecond);

} // namespace app
} // namespace cosmos
