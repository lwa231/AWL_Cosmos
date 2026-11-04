#include <doctest.h>
#include <cmath>
#include <random>
#include "app/units.h"
#include "core/constants.h"
#include "core/kinds.h"

using namespace cosmos;
using namespace cosmos::app;

// Formatting a value back into a field string (plan 10.2)
TEST_SUITE("app units format")
{
    TEST_CASE("formatNumber")
    {
        CHECK(formatNumber(0.0) == "0");
        CHECK(formatNumber(6371.0) == "6371");
        CHECK(formatNumber(0.5) == "0.5");
        CHECK(formatNumber(-2.5) == "-2.5");
        CHECK(formatNumber(123456.0) == "123456");
        CHECK(formatNumber(1234567.0) == "1.23457e6");
        CHECK(formatNumber(1.0e6) == "1e6");
        CHECK(formatNumber(7.35e22) == "7.35e22");
        CHECK(formatNumber(0.001234) == "0.001234");
        CHECK(formatNumber(1.0e-4) == "1e-4");
        CHECK(formatNumber(-3.0e-7) == "-3e-7");
        CHECK(formatNumber(999999.5) == "1e6");   // rounds up across the notation switch
        CHECK(formatNumber(9.9999, 3) == "10");
        CHECK(formatNumber(1234.5678, 3) == "1230");
        CHECK(formatNumber(2.0 / 3.0, 4) == "0.6667");
        CHECK(formatNumber(std::nan("")) == "nan");
    }

    TEST_CASE("lengths pick a unit from the size")
    {
        CHECK(formatQuantity(2.0, Quantity::Length) == "2 m");
        CHECK(formatQuantity(620000.0, Quantity::Length) == "620 km");
        CHECK(formatQuantity(6.371e6, Quantity::Length) == "6371 km");
        CHECK(formatQuantity(6.957e8, Quantity::Length) == "695.7 Mm");
        CHECK(formatQuantity(1.45e10, Quantity::Length) == "14.5 Gm");
        CHECK(formatQuantity(5.0e9, Quantity::Length) == "5 Gm");
        CHECK(formatQuantity(constants::AU, Quantity::Length, nullptr, 4) == "149.6 Gm");
        CHECK(formatQuantity(4.5e12, Quantity::Length, nullptr, 3) == "30.1 AU");
    }

    TEST_CASE("lengths: forced units, negatives, zero")
    {
        CHECK(formatQuantity(1.5e9, Quantity::Length, "AU", 3) == "0.01 AU");
        CHECK(formatQuantity(1.45e10, Quantity::Length, "km") == "1.45e7 km");
        CHECK(formatQuantity(-6.371e6, Quantity::Length) == "-6371 km");
        CHECK(formatQuantity(0.0, Quantity::Length) == "0 m");
    }

    TEST_CASE("masses: M_sun, M_jup, M_earth or kg")
    {
        CHECK(formatQuantity(kSolarMu, Quantity::BodyMass) == "1 M_sun");
        CHECK(formatQuantity(gmEarth(), Quantity::BodyMass) == "1 M_earth");
        CHECK(formatQuantity(gmJupiter(), Quantity::BodyMass) == "1 M_jup");
        CHECK(formatQuantity(1.4 * kSolarMu, Quantity::BodyMass, nullptr, 4) == "1.4 M_sun");
        // The Moon is far below a tenth of an Earth and reads best in kilograms
        CHECK(formatQuantity(7.35e22 * constants::G, Quantity::BodyMass, nullptr, 3) == "7.35e22 kg");
        CHECK(formatQuantity(1.0e15 * constants::G, Quantity::BodyMass, nullptr, 3) == "1e15 kg");
        // The default Earth of table 12.1 is within 0.005% of a standard Earth mass
        CHECK(formatQuantity(5.972e24 * constants::G, Quantity::BodyMass, nullptr, 4) == "1 M_earth");
        CHECK(formatQuantity(3.986e14, Quantity::BodyMass, "m^3/s^2") == "3.986e14 m^3/s^2");
    }

    TEST_CASE("times, speeds, angles, forces")
    {
        CHECK(formatQuantity(30.0, Quantity::Time) == "30 s");
        CHECK(formatQuantity(90.0, Quantity::Time) == "1.5 min");
        CHECK(formatQuantity(5.8 * 3600.0, Quantity::Time) == "5.8 h");
        CHECK(formatQuantity(480.0 * 3600.0, Quantity::Time) == "20 d");
        CHECK(formatQuantity(-1560.0 * 3600.0, Quantity::Time, "h") == "-1560 h");
        CHECK(formatQuantity(2.0 * 365.25 * 86400.0, Quantity::Time) == "2 yr");
        CHECK(formatQuantity(300.0, Quantity::Speed) == "300 m/s");
        CHECK(formatQuantity(7800.0, Quantity::Speed) == "7.8 km/s");
        CHECK(formatQuantity(103.0, Quantity::Angle, "deg") == "103 deg");
        CHECK(formatQuantity(60000.0, Quantity::Force) == "60 kN");
        CHECK(formatQuantity(5000.0, Quantity::Torque) == "5 kN m");
        CHECK(formatQuantity(1.0e4, Quantity::Inertia) == "10000 kg m^2");
        CHECK(formatQuantity(2.0, Quantity::Area) == "2 m^2");
        CHECK(formatQuantity(0.3, Quantity::Number) == "0.3");
    }

    TEST_CASE("an unknown forced unit falls back to the automatic choice")
    {
        CHECK(formatQuantity(6.371e6, Quantity::Length, "furlong") == "6371 km");
    }

    TEST_CASE("what is formatted parses back (to the digits shown)")
    {
        std::mt19937_64 rng(12345);
        std::uniform_real_distribution<double> mant(1.0, 9.99);
        std::uniform_int_distribution<int> expo(-3, 13);
        const Quantity qs[] = {Quantity::Length, Quantity::BodyMass, Quantity::Mass,    Quantity::Time,    Quantity::Speed,
                               Quantity::Force,  Quantity::Torque,   Quantity::Inertia, Quantity::Area,    Quantity::Density};
        for (Quantity q : qs) {
            for (int k = 0; k < 400; ++k) {
                const double v = mant(rng) * std::pow(10.0, expo(rng)) * (k % 7 == 0 ? -1.0 : 1.0);
                const std::string text = formatQuantity(v, q, nullptr, 6);
                const ParseResult r = parseQuantity(text, q);
                INFO("value " << v << " text '" << text << "' error " << r.error);
                REQUIRE(r.ok);
                CHECK(std::fabs(r.value - v) <= 1.0e-5 * std::fabs(v));
            }
        }
        for (double deg : {0.0, 12.5, -90.0, 359.999, 103.0}) {
            const ParseResult r = parseQuantity(formatQuantity(deg, Quantity::Angle, "deg", 9), Quantity::Angle);
            REQUIRE(r.ok);
            CHECK(r.value == doctest::Approx(deg).epsilon(1e-8));
        }
    }
}
