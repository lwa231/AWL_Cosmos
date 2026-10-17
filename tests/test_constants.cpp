#include <doctest.h>
#include "core/constants.h"

using namespace cosmos::constants;

TEST_SUITE("constants")
{
    TEST_CASE("defined values")
    {
        CHECK(c == 299792458.0);
        CHECK(AU == 149597870700.0);
        CHECK(G == 6.67430e-11);
    }

    TEST_CASE("light travel time across 1 AU")
    {
        CHECK(AU / c == doctest::Approx(499.004783836).epsilon(1e-9).scale(0));
    }

    TEST_CASE("G times Earth mass is close to Earth mu")
    {
        // G is known to ~2e-5, so mass-based mu is looser than mu itself
        CHECK(G * 5.972e24 == doctest::Approx(3.986004418e14).epsilon(1e-3).scale(0));
    }
}
