#include <doctest.h>
#include <cmath>
#include "core/constants.h"
#include "core/radii.h"

using namespace cosmos;

namespace {
const double kPi = 3.141592653589793238462643383279502884;
const double muLucerna = 1.25e18;
const double muDomus = 3.767e12;
}

TEST_SUITE("radii")
{
    TEST_CASE("circular speed")
    {
        CHECK(circularSpeed(4.0, 1.0) == 2.0);
        CHECK(circularSpeed(1.0, 4.0) == 0.5);
    }

    TEST_CASE("Domus low orbit at 100 km altitude (plan section 13)")
    {
        const double r = 6.2e5 + 1.0e5;
        const double v = circularSpeed(muDomus, r);
        CHECK(v == doctest::Approx(2287.0).epsilon(1e-3).scale(0));
        const double periodMin = 2.0 * kPi * r / v / 60.0;
        CHECK(periodMin == doctest::Approx(33.0).epsilon(1e-2).scale(0));
    }

    TEST_CASE("Hill radius")
    {
        CHECK(hillRadius(2.0, 0.5, 3.0, 8.0) == doctest::Approx(0.5).epsilon(1e-14).scale(0));
        CHECK(hillRadius(1.0, 0.0, 0.0, 1.0) == 0.0);
    }

    TEST_CASE("Hill radii of Lucerna's planets at a and at periapsis (plan section 13)")
    {
        CHECK(hillRadius(1.45e10, 0.0, muDomus, muLucerna) == doctest::Approx(1.452e8).epsilon(1e-3).scale(0));
        CHECK(hillRadius(1.45e10, 0.015, muDomus, muLucerna) == doctest::Approx(1.430e8).epsilon(1e-3).scale(0));
        CHECK(hillRadius(2.15e10, 0.0, 3.70e11, muLucerna) == doctest::Approx(9.94e7).epsilon(1e-3).scale(0));
        CHECK(hillRadius(2.15e10, 0.06, 3.70e11, muLucerna) == doctest::Approx(9.34e7).epsilon(1e-3).scale(0));
        CHECK(hillRadius(5.8e10, 0.0, 3.05e14, muLucerna) == doctest::Approx(2.513e9).epsilon(1e-3).scale(0));
        CHECK(hillRadius(5.8e10, 0.035, 3.05e14, muLucerna) == doctest::Approx(2.425e9).epsilon(1e-3).scale(0));
    }

    TEST_CASE("sphere of influence")
    {
        CHECK(soiRadius(1.0, 1.0, 32.0) == doctest::Approx(0.25).epsilon(1e-14).scale(0));
        CHECK(soiRadius(4.0, 0.0, 1.0) == 0.0);
    }

    TEST_CASE("mean density and Roche limit")
    {
        const double mass = 5000.0;
        const double mu = constants::G * mass;
        CHECK(meanDensity(mu, 1.0) == doctest::Approx(mass / (4.0 / 3.0 * kPi)).epsilon(1e-12).scale(0));
        CHECK(rocheLimit(1.0, 8.0, 1.0) == doctest::Approx(4.88).epsilon(1e-14).scale(0));
        CHECK(rocheLimit(2.0, 1.0, 1.0) == doctest::Approx(4.88).epsilon(1e-14).scale(0));
    }

    TEST_CASE("mutual Hill radius")
    {
        CHECK(mutualHillRadius(1.0, 2.0, 1.0, 4.0, 8.0) == doctest::Approx(6.0).epsilon(1e-14).scale(0));
    }

    TEST_CASE("Favilla-Velum gap is 47.6 mutual Hill radii (plan section 13)")
    {
        const double rmh = mutualHillRadius(1.79e11, 5.45e12, muLucerna, 5.0e9, 9.4e9);
        const double apoapsisInner = 5.0e9 * (1.0 + 0.08);
        const double periapsisOuter = 9.4e9 * (1.0 - 0.008);
        CHECK((periapsisOuter - apoapsisInner) / rmh == doctest::Approx(47.6).epsilon(2e-3).scale(0));
    }

    TEST_CASE("breakup spin")
    {
        CHECK(breakupSpin(1.0, 1.0) == 1.0);
        CHECK(breakupSpin(8.0, 2.0) == 1.0);
    }
}
