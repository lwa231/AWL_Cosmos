#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "core/kepler.h"

using namespace cosmos;

namespace {
const double kPi = 3.141592653589793238462643383279502884;
const double kTol = 1e-12;

bool near(double x, double ref, double tol = kTol)
{
    return std::abs(x - ref) <= tol * std::max(1.0, std::abs(ref));
}

Vec3 rotate(const Vec3& u, double ang)
{
    const double c = std::cos(ang), s = std::sin(ang);
    return {c * u.x - s * u.y, s * u.x + c * u.y, 0.0};
}

// Ellipse e = 0.5, a = 1, mu = 1, counter-clockwise, at true anomaly pi/2 (periapsis along +x)
const Vec3 kQuarterR{0.0, 0.75, 0.0};
const Vec3 kQuarterV{-1.154700538379251, 0.577350269189626, 0.0};
}

TEST_SUITE("kepler")
{
    TEST_CASE("circular prograde orbit")
    {
        Elements el = stateToElements({1, 0, 0}, {0, 1, 0}, 1.0);
        CHECK(near(el.a, 1.0));
        CHECK(el.e == 0.0);
        CHECK(near(el.h, 1.0));
        CHECK(near(el.p, 1.0));
        CHECK(near(el.q, 1.0));
        CHECK(el.omega == 0.0);
        CHECK(el.nu == 0.0);
        CHECK(el.M == 0.0);
        CHECK_FALSE(el.retrograde);
    }

    TEST_CASE("circular orbit: omega points at the body")
    {
        Elements el = stateToElements({0, 2, 0}, {-std::sqrt(0.5), 0, 0}, 1.0);
        CHECK(near(el.a, 2.0));
        CHECK(el.e == 0.0);
        CHECK(near(el.omega, kPi / 2));
        CHECK(el.nu == 0.0);
        CHECK_FALSE(el.retrograde);
    }

    TEST_CASE("circular retrograde orbit")
    {
        Elements el = stateToElements({1, 0, 0}, {0, -1, 0}, 1.0);
        CHECK(near(el.a, 1.0));
        CHECK(el.e == 0.0);
        CHECK(near(el.h, -1.0));
        CHECK(el.retrograde);
        CHECK(el.omega == 0.0);
        CHECK(el.nu == 0.0);
    }

    TEST_CASE("circular orbit at SI scale (Domus, 100 km altitude)")
    {
        const double mu = 3.767e12, r = 7.2e5;
        Elements el = stateToElements({r, 0, 0}, {0, std::sqrt(mu / r), 0}, mu);
        CHECK(el.e == 0.0);
        CHECK(el.a == doctest::Approx(r).epsilon(1e-12).scale(0));
        CHECK(el.q == doctest::Approx(r).epsilon(1e-12).scale(0));
    }

    TEST_CASE("ellipse at periapsis")
    {
        Elements el = stateToElements({0.5, 0, 0}, {0, std::sqrt(3.0), 0}, 1.0);
        CHECK(near(el.a, 1.0));
        CHECK(near(el.e, 0.5));
        CHECK(near(el.p, 0.75));
        CHECK(near(el.q, 0.5));
        CHECK(near(el.omega, 0.0));
        CHECK(near(el.nu, 0.0));
        CHECK(near(el.M, 0.0));
    }

    TEST_CASE("ellipse at apoapsis")
    {
        Elements el = stateToElements({-1.5, 0, 0}, {0, -std::sqrt(1.0 / 3.0), 0}, 1.0);
        CHECK(near(el.a, 1.0));
        CHECK(near(el.e, 0.5));
        CHECK(near(el.omega, 0.0));
        CHECK(near(el.nu, kPi));
        CHECK(near(el.M, kPi));
    }

    TEST_CASE("ellipse at true anomaly pi/2")
    {
        Elements el = stateToElements(kQuarterR, kQuarterV, 1.0);
        CHECK(near(el.a, 1.0));
        CHECK(near(el.e, 0.5));
        CHECK(near(el.omega, 0.0));
        CHECK(near(el.nu, kPi / 2));
        CHECK(near(el.M, 0.614184849304379));
        CHECK_FALSE(el.retrograde);
    }

    TEST_CASE("mirrored (retrograde) ellipse has the same shape and anomalies")
    {
        Vec3 r{kQuarterR.x, -kQuarterR.y, 0};
        Vec3 v{kQuarterV.x, -kQuarterV.y, 0};
        Elements el = stateToElements(r, v, 1.0);
        CHECK(el.retrograde);
        CHECK(near(el.a, 1.0));
        CHECK(near(el.e, 0.5));
        CHECK(near(el.omega, 0.0));
        CHECK(near(el.nu, kPi / 2));
        CHECK(near(el.M, 0.614184849304379));
    }

    TEST_CASE("rotating the state adds to omega and wraps it")
    {
        Elements a = stateToElements(rotate(kQuarterR, 0.7), rotate(kQuarterV, 0.7), 1.0);
        CHECK(near(a.omega, 0.7));
        CHECK(near(a.nu, kPi / 2));
        CHECK(near(a.M, 0.614184849304379));
        CHECK(near(a.e, 0.5));

        Elements b = stateToElements(rotate(kQuarterR, 4.0), rotate(kQuarterV, 4.0), 1.0);
        CHECK(near(b.omega, 4.0 - 2.0 * kPi));
        CHECK(near(b.nu, kPi / 2));
    }

    TEST_CASE("hyperbola at periapsis")
    {
        Elements el = stateToElements({1, 0, 0}, {0, 1.5, 0}, 1.0);
        CHECK(near(el.a, -4.0));
        CHECK(near(el.e, 1.25));
        CHECK(near(el.p, 2.25));
        CHECK(near(el.q, 1.0));
        CHECK(near(el.nu, 0.0));
        CHECK(near(el.M, 0.0));
    }

    TEST_CASE("hyperbola at true anomaly pi/3")
    {
        Elements el = stateToElements({0.692307692307692, 1.199112097547684, 0},
                                      {-0.577350269189626, 1.166666666666667, 0}, 1.0);
        CHECK(near(el.a, -4.0, 1e-10));
        CHECK(near(el.e, 1.25, 1e-10));
        CHECK(near(el.nu, kPi / 3, 1e-10));
        CHECK(near(el.M, 0.109869533805056, 1e-10));
    }

    TEST_CASE("z components are ignored")
    {
        Elements flat = stateToElements(kQuarterR, kQuarterV, 1.0);
        Elements tilted = stateToElements({kQuarterR.x, kQuarterR.y, 5.0}, {kQuarterV.x, kQuarterV.y, -3.0}, 1.0);
        CHECK(flat.a == tilted.a);
        CHECK(flat.e == tilted.e);
        CHECK(flat.M == tilted.M);
    }
}
