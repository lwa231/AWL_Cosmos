#include "app/units.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "app/display_prefs.h"
#include "core/constants.h"
#include "core/kinds.h"

namespace cosmos {
namespace app {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

// Standard gravitational parameters, m^3/s^2 (JPL DE440 header values for the planets; the Sun's is the project's
// kSolarMu so that 1 M_sun is exactly the default star).
constexpr double kGmEarth = 3.986004418e14;
constexpr double kGmJupiter = 1.26686534e17;

struct Unit {
    const char* name;      // as shown; also parsed (after normalisation)
    double factor;         // canonical units per unit
    const char* aliases;   // '|' separated, may be empty
};

constexpr double kJulianYear = 365.25 * 86400.0;

const Unit kLength[] = {
    {"m", 1.0, ""},
    {"km", 1.0e3, ""},
    {"Mm", 1.0e6, ""},
    {"Gm", 1.0e9, ""},
    {"AU", constants::AU, "au|ua"},
    {"R_earth", 6.371e6, "Rearth|R_E"},
    {"R_jup", 7.1492e7, "Rjup|R_J|R_jupiter"},
    {"R_sun", 6.957e8, "Rsun|R_S|R_solar"},
};

// The canonical unit of BodyMass is mu (m^3/s^2); mass units convert through G or the standard gravitational parameters.
const Unit kBodyMass[] = {
    {"kg", constants::G, ""},
    {"g", constants::G * 1.0e-3, ""},
    {"t", constants::G * 1.0e3, "tonne|tonnes"},
    {"M_earth", kGmEarth, "Mearth|M_E"},
    {"M_jup", kGmJupiter, "Mjup|M_J|M_jupiter"},
    {"M_sun", kSolarMu, "Msun|M_S|M_solar"},
    {"m^3/s^2", 1.0, "m3/s2"},
    {"km^3/s^2", 1.0e9, "km3/s2"},
};

const Unit kMass[] = {
    {"kg", 1.0, ""},
    {"g", 1.0e-3, ""},
    {"t", 1.0e3, "tonne|tonnes"},
};

const Unit kTime[] = {
    {"s", 1.0, "sec|second|seconds"},
    {"min", 60.0, "minute|minutes|mins"},
    {"h", 3600.0, "hr|hour|hours"},
    {"d", 86400.0, "day|days"},
    {"yr", kJulianYear, "y|year|years"},
};

const Unit kSpeed[] = {
    {"m/s", 1.0, "mps"},
    {"km/s", 1.0e3, "kps"},
    {"km/h", 1.0e3 / 3600.0, "kmh|kph"},
};

const Unit kAngle[] = {
    {"deg", 1.0, "\xC2\xB0|degree|degrees"},
    {"rad", 180.0 / kPi, "radian|radians"},
};

const Unit kForce[] = {
    {"N", 1.0, ""},
    {"kN", 1.0e3, ""},
    {"MN", 1.0e6, ""},
};

const Unit kTorque[] = {
    {"N m", 1.0, ""},
    {"kN m", 1.0e3, ""},
};

const Unit kInertia[] = {
    {"kg m^2", 1.0, ""},
    {"t m^2", 1.0e3, ""},
};

const Unit kArea[] = {
    {"m^2", 1.0, ""},
    {"km^2", 1.0e6, ""},
};

const Unit kDensity[] = {
    {"kg/m^3", 1.0, ""},
    {"g/cm^3", 1.0e3, "g/cc"},
};

struct Table {
    const Unit* units;
    std::size_t count;
};

template <std::size_t N>
constexpr Table table(const Unit (&u)[N])
{
    return {u, N};
}

Table tableOf(Quantity q)
{
    switch (q) {
    case Quantity::Length: return table(kLength);
    case Quantity::BodyMass: return table(kBodyMass);
    case Quantity::Mass: return table(kMass);
    case Quantity::Time: return table(kTime);
    case Quantity::Speed: return table(kSpeed);
    case Quantity::Angle: return table(kAngle);
    case Quantity::Force: return table(kForce);
    case Quantity::Torque: return table(kTorque);
    case Quantity::Inertia: return table(kInertia);
    case Quantity::Area: return table(kArea);
    case Quantity::Density: return table(kDensity);
    case Quantity::Number: break;
    }
    return {nullptr, 0};
}

// Index of the canonical unit (factor 1) in the table
std::size_t canonicalIndex(Quantity q)
{
    return q == Quantity::BodyMass ? 6 : 0;   // m^3/s^2
}

constexpr Quantity kAllQuantities[] = {Quantity::Length, Quantity::BodyMass, Quantity::Mass,    Quantity::Time,
                                       Quantity::Speed,  Quantity::Angle,    Quantity::Force,   Quantity::Torque,
                                       Quantity::Inertia, Quantity::Area,    Quantity::Density};

const char* exampleOf(Quantity q)
{
    switch (q) {
    case Quantity::Length: return "13.6 Gm";
    case Quantity::BodyMass: return "5.97e24 kg";
    case Quantity::Mass: return "2000 kg";
    case Quantity::Time: return "5.8 h";
    case Quantity::Speed: return "7.8 km/s";
    case Quantity::Angle: return "90 deg";
    case Quantity::Force: return "60 kN";
    case Quantity::Torque: return "5 kN m";
    case Quantity::Inertia: return "1e4 kg m^2";
    case Quantity::Area: return "2 m^2";
    case Quantity::Density: return "1.2 kg/m^3";
    case Quantity::Number: break;
    }
    return "0.5";
}

// ---- unit name matching --------------------------------------------------------------------------------------------

// Drops spaces, '*', '^' and the middle dot; superscript 2 and 3 become digits
std::string normalise(std::string_view s)
{
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c == ' ' || c == '\t' || c == '*' || c == '^') continue;
        if (c == 0xC2 && i + 1 < s.size()) {
            const unsigned char d = static_cast<unsigned char>(s[i + 1]);
            if (d == 0xB7) { ++i; continue; }
            if (d == 0xB2) { out += '2'; ++i; continue; }
            if (d == 0xB3) { out += '3'; ++i; continue; }
        }
        out += static_cast<char>(c);
    }
    return out;
}

std::string lower(const std::string& s)
{
    std::string out = s;
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

// Calls f(candidate) for the name and every alias of the unit (normalised); stops when f returns true.
template <class F>
bool anyName(const Unit& u, F f)
{
    if (f(normalise(u.name))) return true;
    const char* p = u.aliases;
    while (*p != '\0') {
        const char* e = std::strchr(p, '|');
        const std::size_t len = e ? static_cast<std::size_t>(e - p) : std::strlen(p);
        if (f(normalise(std::string_view(p, len)))) return true;
        if (!e) break;
        p = e + 1;
    }
    return false;
}

// Index of the unit named `token` (already normalised), or -1. Exact match first, then a case-insensitive match for names of
// three or more letters when it identifies exactly one unit.
int findUnit(const Table& t, const std::string& token)
{
    for (std::size_t i = 0; i < t.count; ++i) {
        if (anyName(t.units[i], [&](const std::string& n) { return n == token; })) return static_cast<int>(i);
    }
    if (token.size() >= 3) {
        const std::string lt = lower(token);
        int found = -1;
        for (std::size_t i = 0; i < t.count; ++i) {
            if (anyName(t.units[i], [&](const std::string& n) { return lower(n) == lt; })) {
                if (found >= 0) return -1;   // ambiguous
                found = static_cast<int>(i);
            }
        }
        return found;
    }
    return -1;
}

// The unit the display preference (Settings > Display & units) asks for, as an index into the table; -1 = automatic.
int preferredUnit(const Table& t, Quantity q)
{
    const DisplayPrefs& p = displayPrefs();
    const char* name = nullptr;
    if (q == Quantity::Length) {
        switch (p.length) {
        case LengthUnit::Metres: name = "m"; break;
        case LengthUnit::Kilometres: name = "km"; break;
        case LengthUnit::AU: name = "AU"; break;
        case LengthUnit::Auto: break;
        }
    } else if (q == Quantity::BodyMass) {
        switch (p.mass) {
        case MassUnit::Kilograms: name = "kg"; break;
        case MassUnit::EarthMasses: name = "M_earth"; break;
        case MassUnit::SolarMasses: name = "M_sun"; break;
        case MassUnit::Auto: break;
        }
    }
    return name ? findUnit(t, name) : -1;
}

std::string trimmed(std::string_view s)
{
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return std::string(s.substr(b, e - b));
}

ParseResult fail(std::string message)
{
    ParseResult r;
    r.ok = false;
    r.error = std::move(message);
    return r;
}

std::string quote(const std::string& s) { return "'" + s + "'"; }

// Index of the unit to show for |v| (canonical units)
std::size_t autoUnit(Quantity q, double a)
{
    switch (q) {
    case Quantity::Length:
        if (a < 1.0e3) return 0;      // m
        if (a < 1.0e7) return 1;      // km
        if (a < 1.0e9) return 2;      // Mm
        if (a < 1.0e12) return 3;     // Gm
        return 4;                     // AU
    case Quantity::BodyMass: {
        const double kg = a / constants::G;
        if (kg >= 0.5 * kSolarMu / constants::G) return 5;       // M_sun
        if (kg >= 0.5 * kGmJupiter / constants::G) return 4;     // M_jup
        if (kg >= 0.1 * kGmEarth / constants::G) return 3;       // M_earth
        return 0;                                                // kg
    }
    case Quantity::Time:
        if (a < 60.0) return 0;
        if (a < 3600.0) return 1;
        if (a < 86400.0) return 2;
        if (a < kJulianYear) return 3;
        return 4;
    case Quantity::Speed: return a < 1.0e3 ? 0 : 1;
    case Quantity::Force:
        if (a < 1.0e3) return 0;
        if (a < 1.0e6) return 1;
        return 2;
    case Quantity::Torque: return a < 1.0e3 ? 0 : 1;
    default: return canonicalIndex(q);
    }
}

} // namespace

const char* quantityName(Quantity q)
{
    switch (q) {
    case Quantity::Number: return "number";
    case Quantity::Length: return "length";
    case Quantity::BodyMass: return "mass";
    case Quantity::Mass: return "mass";
    case Quantity::Time: return "time";
    case Quantity::Speed: return "speed";
    case Quantity::Angle: return "angle";
    case Quantity::Force: return "force";
    case Quantity::Torque: return "torque";
    case Quantity::Inertia: return "moment of inertia";
    case Quantity::Area: return "area";
    case Quantity::Density: return "density";
    }
    return "value";
}

const char* canonicalUnit(Quantity q)
{
    const Table t = tableOf(q);
    return t.count == 0 ? "" : t.units[canonicalIndex(q)].name;
}

std::string unitList(Quantity q)
{
    const Table t = tableOf(q);
    std::string out;
    for (std::size_t i = 0; i < t.count; ++i) {
        if (i > 0) out += ", ";
        out += t.units[i].name;
    }
    return out;
}

double gmEarth() { return kGmEarth; }
double gmJupiter() { return kGmJupiter; }
double gmSun() { return kSolarMu; }
double muFromKg(double kg) { return constants::G * kg; }
double kgFromMu(double mu) { return mu / constants::G; }

ParseResult parseQuantity(std::string_view textIn, Quantity q, const char* defaultUnit)
{
    const std::string text = trimmed(textIn);
    const Table tab = tableOf(q);
    if (text.empty()) return fail(std::string("Enter a ") + quantityName(q) + ", for example " + exampleOf(q));

    const std::size_t n = text.size();
    std::size_t i = 0;
    if (text[i] == '+' || text[i] == '-') ++i;
    int digits = 0;
    while (i < n && std::isdigit(static_cast<unsigned char>(text[i]))) { ++i; ++digits; }
    if (i < n && text[i] == '.') {
        ++i;
        while (i < n && std::isdigit(static_cast<unsigned char>(text[i]))) { ++i; ++digits; }
    }
    if (digits == 0) {
        if (text.find(',') != std::string::npos) return fail("Use '.' as the decimal point: " + quote(text) + " is not a number");
        return fail(quote(text) + " is not a number; enter a " + quantityName(q) + " such as " + exampleOf(q));
    }
    if (i < n && (text[i] == 'e' || text[i] == 'E')) {
        std::size_t j = i + 1;
        if (j < n && (text[j] == '+' || text[j] == '-')) ++j;
        std::size_t k = j;
        while (k < n && std::isdigit(static_cast<unsigned char>(text[k]))) ++k;
        if (k > j) {
            i = k;   // exponent
        } else if (j > i + 1 || j >= n) {
            return fail(quote(text.substr(0, j)) + " is not a number: the exponent has no digits");
        }
        // otherwise the 'e' starts a unit name, which is reported below
    }
    const std::string numberText = text.substr(0, i);
    errno = 0;
    char* endp = nullptr;
    const double mantissa = std::strtod(numberText.c_str(), &endp);
    if (endp == numberText.c_str() || errno == ERANGE || !std::isfinite(mantissa))
        return fail(quote(numberText) + " is out of range");

    const std::string rest = trimmed(std::string_view(text).substr(i));
    if (!rest.empty() && rest[0] == ',') return fail("Use '.' as the decimal point (and no thousands separators): " + quote(text));
    double factor = 1.0;
    if (q == Quantity::Number) {
        if (!rest.empty()) return fail("This field takes a plain number, not " + quote(rest));
    } else if (rest.empty()) {
        if (defaultUnit != nullptr && defaultUnit[0] != '\0') {
            const int d = findUnit(tab, normalise(defaultUnit));
            if (d < 0)
                return fail(std::string("Internal error: default unit ") + quote(defaultUnit) + " is not a " + quantityName(q) + " unit");
            factor = tab.units[d].factor;
        } else {
            factor = tab.units[canonicalIndex(q)].factor;
        }
    } else {
        const std::string token = normalise(rest);
        const int u = findUnit(tab, token);
        if (u < 0) {
            std::string msg = "Unknown unit " + quote(rest) + " for a " + quantityName(q);
            // A unit that exists for another quantity gets a more helpful message
            for (Quantity other : kAllQuantities) {
                if (other == q) continue;
                const Table ot = tableOf(other);
                bool exact = false;
                for (std::size_t k = 0; k < ot.count && !exact; ++k)
                    exact = anyName(ot.units[k], [&](const std::string& nm) { return nm == token; });
                if (exact) {
                    msg = quote(rest) + " is a unit of " + quantityName(other) + ", but this field needs a " + quantityName(q);
                    break;
                }
            }
            return fail(msg + "; use " + unitList(q));
        }
        factor = tab.units[static_cast<std::size_t>(u)].factor;
    }

    const double value = mantissa * factor;
    if (!std::isfinite(value)) return fail(quote(text) + " is out of range");
    ParseResult r;
    r.ok = true;
    r.value = value;
    return r;
}

std::string formatNumber(double value, int sig)
{
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0.0 ? "-inf" : "inf";
    if (sig < 1) sig = 1;
    if (sig > 15) sig = 15;
    if (value == 0.0) return "0";

    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*e", sig - 1, value);   // rounds first, so the exponent is the one that is shown
    const char* e = std::strchr(buf, 'e');
    const int exp10 = e ? std::atoi(e + 1) : 0;

    const auto trim = [](std::string s) {
        if (s.find('.') != std::string::npos) {
            while (!s.empty() && s.back() == '0') s.pop_back();
            if (!s.empty() && s.back() == '.') s.pop_back();
        }
        return s;
    };

    if (exp10 >= -3 && exp10 <= 5) {
        const int decimals = std::max(0, sig - 1 - exp10);
        char fixed[64];
        // The value rounded to sig digits (so 1234.5678 at 3 digits reads 1230, not 1235)
        std::snprintf(fixed, sizeof fixed, "%.*f", decimals, std::strtod(buf, nullptr));
        return trim(fixed);
    }
    const std::string mant = trim(std::string(buf, static_cast<std::size_t>(e - buf)));
    return mant + "e" + std::to_string(exp10);
}

std::string formatQuantity(double value, Quantity q, const char* forceUnit, int sig)
{
    if (q == Quantity::Number || !std::isfinite(value)) return formatNumber(value, sig);
    const Table tab = tableOf(q);
    int idx = -1;
    if (forceUnit != nullptr && forceUnit[0] != '\0') idx = findUnit(tab, normalise(forceUnit));
    if (idx < 0) idx = preferredUnit(tab, q);
    const Unit& u = idx >= 0 ? tab.units[static_cast<std::size_t>(idx)] : tab.units[autoUnit(q, std::fabs(value))];
    return formatNumber(value / u.factor, sig) + " " + u.name;
}

} // namespace app
} // namespace cosmos
