#include "app/format.h"
#include "app/display_prefs.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace cosmos {
namespace app {

namespace {

constexpr double kAU = 149597870700.0;        // m (IAU 2012; same as cosmos::constants::AU, repeated to keep this file standalone)
constexpr double kJulianYear = 365.25 * 86400.0;

// // Pick the unit for |v|: the last one whose threshold is reached, then move up while the value as it would be DISPLAYED
// in the next unit reaches that unit's threshold (so "1000 m" never shows, it is "1.00 km"; but 59.96 s stays "60.0 s"
// because in minutes it would read 0.999). rounded to `sig` significant figures, as a number (used to decide unit changes on the value that is displayed)
double roundSig(double v, int sig)
{
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.*e", sig - 1, v);
    return std::strtod(buf, nullptr);
}

struct Unit {
    const char* name;
    double size;        // metres / seconds per unit
    double threshold;   // smallest magnitude (in base units) shown in this unit
};

// Pick the unit for |v|: the last one whose threshold is reached, then move up while the value as it would be DISPLAYED
// in the next unit reaches that unit's threshold ("1000 m" never shows, it is "1.00 km"; 59.96 s stays "60.0 s" because
// in minutes it would read 0.999).
template <std::size_t N>
const Unit& pickUnit(const Unit (&units)[N], double absValue, int sig)
{
    std::size_t k = 0;
    while (k + 1 < N && absValue >= units[k + 1].threshold) ++k;
    while (k + 1 < N &&
           roundSig(absValue / units[k + 1].size, sig) >= roundSig(units[k + 1].threshold / units[k + 1].size, sig))
        ++k;
    return units[k];
}

std::string withUnit(double signedValue, const Unit& u, const char* suffix, int sig)
{
    std::string s = formatSig(signedValue / u.size, sig);
    s += ' ';
    s += u.name;
    s += suffix;
    return s;
}

} // namespace

std::string formatSig(double value, int sig)
{
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0.0 ? "-inf" : "inf";
    if (sig < 1) sig = 1;
    if (sig > 9) sig = 9;
    char buf[64];
    if (value == 0.0) {
        // "0", "0.0", "0.00", ... so that the digit count looks the same as for other values
        return sig == 1 ? "0" : "0." + std::string(static_cast<std::size_t>(sig - 1), '0');
    }
    // The exponent after rounding decides the decimals (9.996 at 3 figures is 1.00e+01, so one decimal: "10.0")
    std::snprintf(buf, sizeof buf, "%.*e", sig - 1, value);
    const char* e = std::strchr(buf, 'e');
    const int exp10 = e ? std::atoi(e + 1) : 0;
    int decimals = sig - 1 - exp10;
    if (decimals < 0) decimals = 0;
    std::snprintf(buf, sizeof buf, "%.*f", decimals, value);
    return buf;
}

std::string formatTimeRate(double rate)
{
    if (!std::isfinite(rate)) return "-- /sec";
    static const Unit units[] = {{"s", 1.0, 0.0}, {"min", 60.0, 60.0}, {"h", 3600.0, 3600.0},
                                 {"days", 86400.0, 86400.0}, {"years", kJulianYear, kJulianYear}};
    const Unit& u = pickUnit(units, std::fabs(rate), 3);
    return withUnit(rate, u, "/sec", 3);
}

std::string formatSimDate(double t) { return formatSimDate(t, displayPrefs().date); }

std::string formatSimDate(double t, DateFormat format)
{
    if (!std::isfinite(t)) return "Day --";
    double a = std::fabs(t);
    if (a > 9.0e15) a = 9.0e15;   // beyond 2^53 whole seconds no longer exist as doubles; keep the integer conversion safe
    const long long total = static_cast<long long>(std::floor(a));
    const bool neg = t < 0.0 && total > 0;
    const long long day = total / 86400;
    const int secOfDay = static_cast<int>(total % 86400);
    const char* sign = neg ? "-" : "";
    char buf[96];
    switch (format) {
    case DateFormat::DayOnly:
        std::snprintf(buf, sizeof buf, "%sDay %lld", sign, day);
        break;
    case DateFormat::YearDay:
        std::snprintf(buf, sizeof buf, "%sYear %lld, Day %lld  %02d:%02d:%02d", sign, day / 365, day % 365, secOfDay / 3600,
                      (secOfDay / 60) % 60, secOfDay % 60);
        break;
    case DateFormat::DayClock:
    default:
        std::snprintf(buf, sizeof buf, "%sDay %lld  %02d:%02d:%02d", sign, day, secOfDay / 3600, (secOfDay / 60) % 60, secOfDay % 60);
        break;
    }
    return buf;
}

std::string formatDistance(double m)
{
    if (!std::isfinite(m)) return formatSig(m, 3) + " m";
    static const Unit units[] = {{"m", 1.0, 0.0}, {"km", 1.0e3, 1.0e3}, {"Mm", 1.0e6, 1.0e6}, {"Gm", 1.0e9, 1.0e9},
                                 {"AU", kAU, 1.0e11}};
    switch (displayPrefs().length) {   // a fixed unit from the settings
    case LengthUnit::Metres: return withUnit(m, units[0], "", 3);
    case LengthUnit::Kilometres: return withUnit(m, units[1], "", 3);
    case LengthUnit::AU: return withUnit(m, units[4], "", 3);
    case LengthUnit::Auto: break;
    }
    return withUnit(m, pickUnit(units, std::fabs(m), 3), "", 3);
}

std::string formatSpeed(double v)
{
    if (!std::isfinite(v)) return formatSig(v, 3) + " m/s";
    static const Unit units[] = {{"m/s", 1.0, 0.0}, {"km/s", 1.0e3, 1.0e3}};
    return withUnit(v, pickUnit(units, std::fabs(v), 3), "", 3);
}

} // namespace app
} // namespace cosmos
