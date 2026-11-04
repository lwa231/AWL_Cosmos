#pragma once
// Units grammar of the editor fields (plan section 10.2). No OpenGL, no ImGui.
//
// A field accepts "<number> <unit>" (the space is optional, the number may use scientific notation, leading and trailing
// white space is ignored) or a bare number, which takes the unit the field says is its default:
//     "1.36e10 m"   "13.6 Gm"   "0.09 AU"   "6371km"   "5.972e24 kg"   "1 M_earth"   "1.5 d"   "7.8 km/s"   "-90"
//
//   Length   m, km, Mm, Gm, AU (also R_earth, R_jup, R_sun)
//   BodyMass kg, g, t, M_earth, M_jup, M_sun, and gravitational-parameter units m^3/s^2, km^3/s^2
//   Mass     kg, g, t                                  (plain mass: shuttle dry mass and fuel)
//   Time     s, min, h, d, yr                          (also sec, hr, day(s), year(s); yr is the Julian year, 365.25 d)
//   Speed    m/s, km/s, km/h
//   Angle    deg (or the degree sign), rad
//   Force    N, kN, MN        Torque  N m, kN m        Inertia  kg m^2, t m^2
//   Area     m^2, km^2        Density kg/m^3, g/cm^3   Number   (no unit)
//
// Every parse returns the value in the quantity's CANONICAL unit: metres, mu = GM in m^3/s^2 for BodyMass (the value that
// is stored, plan Q13/5.1), kilograms, seconds, m/s, DEGREES for Angle (the editor works in degrees, as the scene file
// does), newtons, N m, kg m^2, m^2, kg/m^3. The mass units of BodyMass are converted to mu through the standard
// gravitational parameters (GM of the Earth, Jupiter and the Sun, so "1 M_sun" is exactly the default star's mu); a plain
// "kg" goes through G. They are approximate as masses, which does not matter because mu is what the physics uses.
//
// Unit names are case sensitive where the case carries meaning (Mm is not mm, Gm is not GM); a unit of three or more
// letters is also matched ignoring case when that is unambiguous ("Days", "m_earth"). Spaces, '*' and '^' inside a unit
// are ignored, so "N*m", "N m" and "Nm" are the same.
#include <string>
#include <string_view>

namespace cosmos {
namespace app {

enum class Quantity { Number, Length, BodyMass, Mass, Time, Speed, Angle, Force, Torque, Inertia, Area, Density };

// "length", "mass", ... for messages.
const char* quantityName(Quantity q);

// The canonical unit as text ("m", "m^3/s^2" for BodyMass, "deg" for Angle, ...); "" for Number.
const char* canonicalUnit(Quantity q);

// "m, km, Mm, Gm, AU, R_earth, R_jup, R_sun" (the primary names), for hints and error messages.
std::string unitList(Quantity q);

struct ParseResult {
    bool ok = false;
    double value = 0.0;   // canonical unit of the quantity
    std::string error;    // readable message, empty when ok
    explicit operator bool() const { return ok; }
};

// Parses a field string. defaultUnit is the unit a bare number takes (one of the quantity's unit names; nullptr or "" =
// the canonical unit). The result is always finite when ok.
ParseResult parseQuantity(std::string_view text, Quantity q, const char* defaultUnit = nullptr);

// "6371 km", "5.97 M_earth", "14.9 d": value (canonical unit) as text with `sig` significant digits (trailing zeros
// removed, so it is a string to edit, not a column to align). forceUnit picks the unit (a unit name of the quantity; an
// unknown name falls back to the automatic choice); nullptr chooses a sensible unit from the magnitude, or the
// unit of displayPrefs() (display_prefs.h) for a Length or BodyMass. parseQuantity
// of the result returns the value to within 10^-(sig-1) relative.
std::string formatQuantity(double value, Quantity q, const char* forceUnit = nullptr, int sig = 6);

// The number alone: fixed notation for 1e-3 <= |v| < 1e6, otherwise scientific as "1.2e22" (no '+', no leading zeros in
// the exponent). Trailing zeros are removed. sig is clamped to 1..15.
std::string formatNumber(double value, int sig = 6);

// Standard gravitational parameters used by the mass units, m^3/s^2 (the Sun's value is the default star's mu).
double gmEarth();
double gmJupiter();
double gmSun();

// mu <-> kg through G (display and input only; mu is what is stored).
double muFromKg(double kg);
double kgFromMu(double mu);

} // namespace app
} // namespace cosmos
