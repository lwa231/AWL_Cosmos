#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include "core/kepler.h"

using namespace cosmos;

namespace {
const double kPi = 3.141592653589793238462643383279502884;

double wrapPi(double a)
{
    double t = std::fmod(a + kPi, 2.0 * kPi);
    if (t <= 0.0) t += 2.0 * kPi;
    return t - kPi;
}

bool near(double x, double ref, double tol)
{
    return std::abs(x - ref) <= tol * std::max(1.0, std::abs(ref));
}

double relErr(const Vec3& a, const Vec3& ref)
{
    return norm(a - ref) / norm(ref);
}

// Fixed-seed, platform-independent uniform in [0, 1)
double uni(std::mt19937_64& g)
{
    return static_cast<double>(g() >> 11) * (1.0 / 9007199254740992.0);
}

struct Case {
    Elements el;
    double mu;
};

Case randomCase(std::mt19937_64& g, int index)
{
    Case c;
    c.mu = std::pow(10.0, 12.0 * uni(g));
    c.el.retrograde = uni(g) < 0.5;
    c.el.omega = (2.0 * uni(g) - 1.0) * kPi;

    const int kind = index % 5;
    if (kind == 4) {
        c.el.e = 1.05 + 2.0 * uni(g);
        c.el.a = -(0.5 + 50.0 * uni(g));
        c.el.M = (2.0 * uni(g) - 1.0) * 3.0;
        return c;
    }
    c.el.a = 0.5 + 50.0 * uni(g);
    c.el.M = (2.0 * uni(g) - 1.0) * kPi;
    switch (kind) {
        case 0: c.el.e = 0.95 * uni(g); break;
        case 1: c.el.e = 0.0; break;
        case 2: c.el.e = std::pow(10.0, -12.0 + 8.0 * uni(g)); break;
        default: c.el.e = 0.5 + 0.45 * uni(g); break;
    }
    return c;
}
}

TEST_SUITE("kepler")
{
    TEST_CASE("elements to state: ellipse at true anomaly pi/2")
    {
        Elements el;
        el.a = 1.0;
        el.e = 0.5;
        el.omega = 0.0;
        el.M = 0.614184849304379;
        RelState s = elementsToState(el, 1.0);
        CHECK(near(s.r.x, 0.0, 1e-12));
        CHECK(near(s.r.y, 0.75, 1e-12));
        CHECK(near(s.v.x, -1.154700538379251, 1e-12));
        CHECK(near(s.v.y, 0.577350269189626, 1e-12));
    }

    TEST_CASE("elements to state: periapsis of the e = 0.5 ellipse")
    {
        Elements el;
        el.a = 1.0;
        el.e = 0.5;
        RelState s = elementsToState(el, 1.0);
        CHECK(near(s.r.x, 0.5, 1e-12));
        CHECK(near(s.r.y, 0.0, 1e-12));
        CHECK(near(s.v.x, 0.0, 1e-12));
        CHECK(near(s.v.y, std::sqrt(3.0), 1e-12));
    }

    TEST_CASE("elements to state: retrograde mirrors y")
    {
        Elements el;
        el.a = 1.0;
        el.e = 0.5;
        el.M = 0.614184849304379;
        el.retrograde = true;
        RelState s = elementsToState(el, 1.0);
        CHECK(near(s.r.y, -0.75, 1e-12));
        CHECK(near(s.v.x, -1.154700538379251, 1e-12));
        CHECK(near(s.v.y, -0.577350269189626, 1e-12));
    }

    TEST_CASE("elements to state: rotation by omega")
    {
        Elements el;
        el.a = 1.0;
        el.e = 0.5;
        el.omega = kPi / 2;
        RelState s = elementsToState(el, 1.0);
        CHECK(near(s.r.x, 0.0, 1e-12));
        CHECK(near(s.r.y, 0.5, 1e-12));
        CHECK(near(s.v.x, -std::sqrt(3.0), 1e-12));
    }

    TEST_CASE("elements to state: hyperbola at periapsis and at nu = pi/3")
    {
        Elements el;
        el.a = -4.0;
        el.e = 1.25;
        RelState p = elementsToState(el, 1.0);
        CHECK(near(p.r.x, 1.0, 1e-12));
        CHECK(near(p.v.y, 1.5, 1e-12));

        el.M = 0.109869533805056;
        RelState s = elementsToState(el, 1.0);
        CHECK(near(s.r.x, 0.692307692307692, 1e-10));
        CHECK(near(s.r.y, 1.199112097547684, 1e-10));
        CHECK(near(s.v.x, -0.577350269189626, 1e-10));
        CHECK(near(s.v.y, 1.166666666666667, 1e-10));
    }

    TEST_CASE("mean anomaly is periodic for an ellipse")
    {
        Elements a;
        a.a = 2.0;
        a.e = 0.7;
        a.M = 0.9;
        Elements b = a;
        b.M = 0.9 + 4.0 * kPi;
        RelState sa = elementsToState(a, 3.0);
        RelState sb = elementsToState(b, 3.0);
        CHECK(relErr(sb.r, sa.r) < 1e-12);
        CHECK(relErr(sb.v, sa.v) < 1e-12);
    }

    TEST_CASE("energy and angular momentum of the generated state match the elements")
    {
        Elements el;
        el.a = 3.0;
        el.e = 0.9;
        el.M = 1.1;
        el.omega = 0.4;
        const double mu = 2.5;
        RelState s = elementsToState(el, mu);
        const double energy = 0.5 * norm2(s.v) - mu / norm(s.r);
        CHECK(near(energy, -mu / (2.0 * el.a), 1e-12));
        const double h = s.r.x * s.v.y - s.r.y * s.v.x;
        CHECK(near(h, std::sqrt(mu * el.a * (1.0 - el.e * el.e)), 1e-12));
    }

    TEST_CASE("round trip over 2000 random orbits")
    {
        std::mt19937_64 g(12345);
        double worstState = 0.0;
        int bad = 0;

        for (int i = 0; i < 2000; ++i) {
            const Case c = randomCase(g, i);
            const RelState s1 = elementsToState(c.el, c.mu);
            const Elements el1 = stateToElements(s1.r, s1.v, c.mu);
            const RelState s2 = elementsToState(el1, c.mu);

            // e below 1e-10 is snapped to 0, which moves the state by about e
            const double allowed = 1e-12 + (c.el.e < 1e-10 ? 2.0 * c.el.e : 0.0);
            const double stateErr = std::max(relErr(s2.r, s1.r), relErr(s2.v, s1.v));
            worstState = std::max(worstState, stateErr - (allowed - 1e-12));
            if (stateErr >= allowed) ++bad;

            const double sgn = c.el.retrograde ? -1.0 : 1.0;
            const double angle0 = c.el.omega + sgn * c.el.M;
            const double angle1 = el1.omega + sgn * el1.M;
            if (c.el.e > 1e-6) {
                CHECK(near(el1.a, c.el.a, 1e-12));
                CHECK(near(el1.e, c.el.e, 1e-12));
                CHECK(el1.retrograde == c.el.retrograde);
            }
            if (c.el.e < 1.0 || c.el.e > 1e-6) {
                CHECK(std::abs(wrapPi(angle1 - angle0)) < 1e-9);
            }
        }
        CHECK(bad == 0);
        CHECK(worstState < 1e-12);
    }
}
