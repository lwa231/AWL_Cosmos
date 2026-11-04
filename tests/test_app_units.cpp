#include <doctest.h>
#include <cmath>
#include "app/units.h"
#include "core/constants.h"
#include "core/kinds.h"

using namespace cosmos;
using namespace cosmos::app;

namespace {
double parseOk(const char* text, Quantity q, const char* def = nullptr)
{
    const ParseResult r = parseQuantity(text, q, def);
    INFO("text: '" << text << "'  error: " << r.error);
    REQUIRE(r.ok);
    return r.value;
}
std::string parseErr(const char* text, Quantity q, const char* def = nullptr)
{
    const ParseResult r = parseQuantity(text, q, def);
    INFO("text: '" << text << "'");
    CHECK_FALSE(r.ok);
    return r.error;
}
bool contains(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }
} // namespace

// Units grammar of the editor fields (plan 10.2)
TEST_SUITE("app units")
{
    TEST_CASE("lengths: the plan examples and every unit")
    {
        CHECK(parseOk("1.36e10 m", Quantity::Length) == 1.36e10);
        CHECK(parseOk("13.6 Gm", Quantity::Length) == doctest::Approx(1.36e10).epsilon(1e-15));
        CHECK(parseOk("0.09 AU", Quantity::Length) == doctest::Approx(0.09 * constants::AU).epsilon(1e-15));
        CHECK(parseOk("6371km", Quantity::Length) == 6.371e6);
        CHECK(parseOk("2.5 Mm", Quantity::Length) == 2.5e6);
        CHECK(parseOk("1 AU", Quantity::Length) == constants::AU);
        CHECK(parseOk("1 au", Quantity::Length) == constants::AU);
        CHECK(parseOk("1 R_earth", Quantity::Length) == 6.371e6);
        CHECK(parseOk("1 R_sun", Quantity::Length) == 6.957e8);
    }

    TEST_CASE("numbers: sign, decimals, exponents, white space")
    {
        CHECK(parseOk("  5 km  ", Quantity::Length) == 5000.0);
        CHECK(parseOk("\t1.5 \t Gm\n", Quantity::Length) == 1.5e9);
        CHECK(parseOk("1E3 m", Quantity::Length) == 1000.0);
        CHECK(parseOk("1e+3 m", Quantity::Length) == 1000.0);
        CHECK(parseOk("2.5e-3 km", Quantity::Length) == doctest::Approx(2.5).epsilon(1e-15));
        CHECK(parseOk("+5 m", Quantity::Length) == 5.0);
        CHECK(parseOk("-5 km", Quantity::Length) == -5000.0);
        CHECK(parseOk(".5 km", Quantity::Length) == 500.0);
        CHECK(parseOk("5. km", Quantity::Length) == 5000.0);
        CHECK(parseOk("1.36e10m", Quantity::Length) == 1.36e10);
        CHECK(parseOk("0", Quantity::Length) == 0.0);
    }

    TEST_CASE("bare numbers take the default unit of the field")
    {
        CHECK(parseOk("6371", Quantity::Length, "km") == 6.371e6);
        CHECK(parseOk("6371", Quantity::Length) == 6371.0);   // canonical unit: metres
        CHECK(parseOk("6371", Quantity::Length, "") == 6371.0);
        CHECK(parseOk("5.8", Quantity::Time, "h") == 5.8 * 3600.0);
        CHECK(parseOk("90", Quantity::Angle) == 90.0);   // degrees
        CHECK(parseOk("2", Quantity::BodyMass, "M_earth") == 2.0 * gmEarth());
        CHECK(parseOk("0.5", Quantity::Number) == 0.5);
        CHECK(parseOk("3", Quantity::Speed, "km/s") == 3000.0);
    }

    TEST_CASE("masses are converted to mu, the stored value")
    {
        CHECK(parseOk("1 M_sun", Quantity::BodyMass) == kSolarMu);   // exactly the default star
        CHECK(parseOk("1 M_earth", Quantity::BodyMass) == gmEarth());
        CHECK(parseOk("1 M_jup", Quantity::BodyMass) == gmJupiter());
        CHECK(parseOk("5.972e24 kg", Quantity::BodyMass) == constants::G * 5.972e24);
        CHECK(parseOk("1000 g", Quantity::BodyMass) == doctest::Approx(constants::G).epsilon(1e-15));
        CHECK(parseOk("1 t", Quantity::BodyMass) == doctest::Approx(constants::G * 1000.0).epsilon(1e-15));
        CHECK(parseOk("3.986e14 m^3/s^2", Quantity::BodyMass) == 3.986e14);
        CHECK(parseOk("3.986e14 m3/s2", Quantity::BodyMass) == 3.986e14);
        CHECK(parseOk("1 km^3/s^2", Quantity::BodyMass) == 1.0e9);
        // The approximate conversions agree with the textbook kilograms
        CHECK(kgFromMu(gmEarth()) == doctest::Approx(5.972e24).epsilon(1e-3));
        CHECK(kgFromMu(gmJupiter()) == doctest::Approx(1.898e27).epsilon(1e-3));
        CHECK(kgFromMu(kSolarMu) == doctest::Approx(1.989e30).epsilon(1e-3));
        CHECK(muFromKg(kgFromMu(1.234e12)) == doctest::Approx(1.234e12).epsilon(1e-15));
    }

    TEST_CASE("plain masses for shuttles")
    {
        CHECK(parseOk("2000 kg", Quantity::Mass) == 2000.0);
        CHECK(parseOk("3 t", Quantity::Mass) == 3000.0);
        CHECK(parseOk("500", Quantity::Mass) == 500.0);
        CHECK_FALSE(parseQuantity("1 M_earth", Quantity::Mass).ok);
    }

    TEST_CASE("times, speeds, angles")
    {
        CHECK(parseOk("90 s", Quantity::Time) == 90.0);
        CHECK(parseOk("2 min", Quantity::Time) == 120.0);
        CHECK(parseOk("1.5 h", Quantity::Time) == 5400.0);
        CHECK(parseOk("1 d", Quantity::Time) == 86400.0);
        CHECK(parseOk("1 yr", Quantity::Time) == 365.25 * 86400.0);
        CHECK(parseOk("2 days", Quantity::Time) == 172800.0);
        CHECK(parseOk("2 Days", Quantity::Time) == 172800.0);
        CHECK(parseOk("-65 d", Quantity::Time) == -65.0 * 86400.0);

        CHECK(parseOk("300 m/s", Quantity::Speed) == 300.0);
        CHECK(parseOk("7.8 km/s", Quantity::Speed) == 7800.0);
        CHECK(parseOk("36 km/h", Quantity::Speed) == doctest::Approx(10.0).epsilon(1e-15));

        CHECK(parseOk("90 deg", Quantity::Angle) == 90.0);
        CHECK(parseOk("45\xC2\xB0", Quantity::Angle) == 45.0);
        CHECK(parseOk("3.14159265358979 rad", Quantity::Angle) == doctest::Approx(180.0).epsilon(1e-12));
        CHECK(parseOk("-90 degrees", Quantity::Angle) == -90.0);
    }

    TEST_CASE("shuttle quantities; spaces and operators inside units do not matter")
    {
        CHECK(parseOk("60 kN", Quantity::Force) == 60000.0);
        CHECK(parseOk("5 kN m", Quantity::Torque) == 5000.0);
        CHECK(parseOk("5 kN*m", Quantity::Torque) == 5000.0);
        CHECK(parseOk("5kNm", Quantity::Torque) == 5000.0);
        CHECK(parseOk("1e4 kg m^2", Quantity::Inertia) == 1.0e4);
        CHECK(parseOk("1e4 kg*m2", Quantity::Inertia) == 1.0e4);
        CHECK(parseOk("2 m^2", Quantity::Area) == 2.0);
        CHECK(parseOk("1.225 kg/m^3", Quantity::Density) == 1.225);
        CHECK(parseOk("1 g/cm3", Quantity::Density) == 1000.0);
    }

    TEST_CASE("unit names: case matters for prefixes, long names are forgiving")
    {
        CHECK(parseOk("1 Mm", Quantity::Length) == 1.0e6);
        CHECK(parseOk("1 Gm", Quantity::Length) == 1.0e9);
        CHECK(parseOk("1 m_EARTH", Quantity::BodyMass) == gmEarth());
        CHECK(parseOk("1 Msun", Quantity::BodyMass) == kSolarMu);
        // mm (millimetres) is not a unit here, and MM / GM must not silently turn into Mm / Gm
        CHECK(contains(parseErr("5 mm", Quantity::Length).c_str(), "Unknown unit 'mm'"));
        CHECK(contains(parseErr("5 MM", Quantity::Length).c_str(), "Unknown unit"));
        CHECK(contains(parseErr("5 GM", Quantity::Length).c_str(), "Unknown unit"));
    }

    TEST_CASE("errors say what is wrong")
    {
        CHECK(contains(parseErr("", Quantity::Length), "Enter a length"));
        CHECK(contains(parseErr("   ", Quantity::Mass), "Enter a mass"));
        CHECK(contains(parseErr("abc", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("km", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("-", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("1e", Quantity::Length), "exponent"));
        CHECK(contains(parseErr("1e+", Quantity::Length), "exponent"));
        CHECK(contains(parseErr("1e- m", Quantity::Length), "exponent"));
        CHECK(contains(parseErr("12 foo", Quantity::Length), "Unknown unit 'foo'"));
        CHECK(contains(parseErr("12 foo", Quantity::Length), "m, km, Mm, Gm, AU"));   // lists what is accepted
        CHECK(contains(parseErr("1,5 km", Quantity::Length), "decimal point"));
        CHECK(contains(parseErr("--5 m", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("1 2 m", Quantity::Length), "Unknown unit"));
        CHECK(contains(parseErr("1e999 m", Quantity::Length), "out of range"));
        CHECK(contains(parseErr("inf", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("nan", Quantity::Length), "not a number"));
        CHECK(contains(parseErr("5 m m", Quantity::Length), "Unknown unit"));
        CHECK(contains(parseErr("5 m", Quantity::Number), "plain number"));
        CHECK(contains(parseErr("5", Quantity::Length, "furlong"), "default unit"));
    }

    TEST_CASE("a unit of another quantity gets a helpful message")
    {
        const std::string e1 = parseErr("5 kg", Quantity::Length);
        CHECK(contains(e1, "unit of mass"));
        CHECK(contains(e1, "needs a length"));
        const std::string e2 = parseErr("5 m", Quantity::Mass);
        CHECK(contains(e2, "unit of length"));
        CHECK(contains(parseErr("5 km/s", Quantity::Length), "unit of speed"));
        CHECK(contains(parseErr("5 d", Quantity::Speed), "unit of time"));
    }

    TEST_CASE("a parse never returns a non-finite value")
    {
        CHECK(std::isfinite(parseOk("1e300 m", Quantity::Length)));
        CHECK_FALSE(parseQuantity("1e300 AU", Quantity::Length).ok);   // 1.5e311 overflows: rejected, not inf
        CHECK(parseOk("-0 m", Quantity::Length) == 0.0);
    }

    TEST_CASE("canonical units and unit lists")
    {
        CHECK(std::string(canonicalUnit(Quantity::Length)) == "m");
        CHECK(std::string(canonicalUnit(Quantity::BodyMass)) == "m^3/s^2");
        CHECK(std::string(canonicalUnit(Quantity::Angle)) == "deg");
        CHECK(std::string(canonicalUnit(Quantity::Number)).empty());
        CHECK(unitList(Quantity::Time) == "s, min, h, d, yr");
        CHECK(unitList(Quantity::Speed) == "m/s, km/s, km/h");
    }
}
