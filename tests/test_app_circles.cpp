#include <doctest.h>
#include "app/overlay_fill.h"
#include "core/constants.h"
#include <cmath>

using namespace cosmos;
using namespace cosmos::app;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A body on a counter-clockwise orbit about a parent, started at periapsis (x axis) so a and e are known exactly.
struct Sys {
    std::vector<Vec3> r, v;
    std::vector<double> mu, radius;
    void add(double m, double rad, Vec3 pos, Vec3 vel)
    {
        r.push_back(pos);
        v.push_back(vel);
        mu.push_back(m);
        radius.push_back(rad);
    }
    // Orbit of (m, rad) about the body at index p, which must sit at rest at the origin of its own frame
    void addOrbiter(std::size_t p, double m, double rad, double a, double e)
    {
        const double q = a * (1.0 - e);
        const double speed = std::sqrt((mu[p] + m) * (1.0 + e) / q);
        add(m, rad, r[p] + Vec3{q, 0.0, 0.0}, v[p] + Vec3{0.0, speed, 0.0});
    }
};

double density(double mu, double radius) { return mu / (constants::G * 4.0 / 3.0 * kPi * radius * radius * radius); }

}

TEST_SUITE("app circles")
{
    TEST_CASE("circular orbit: Hill radius a cbrt(m / 3M) and SOI radius a (m / M)^(2/5)")
    {
        Sys s;
        s.add(1.327e20, 6.96e8, {0, 0, 0}, {0, 0, 0});
        const double a = 1.496e11, m = 3.986e14;
        s.addOrbiter(0, m, 6.371e6, a, 0.0);
        const auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);
        REQUIRE(radii.size() == 2);
        CHECK(radii[1].parent == 0);
        CHECK(radii[1].hill == doctest::Approx(a * std::cbrt(m / (3.0 * 1.327e20))).epsilon(1e-9));
        CHECK(radii[1].soi == doctest::Approx(a * std::pow(m / 1.327e20, 0.4)).epsilon(1e-9));
        CHECK(radii[1].hill > 1.4e9);   // Earth's Hill sphere is about 1.5 million km
        CHECK(radii[1].hill < 1.6e9);
        CHECK(radii[1].soi == doctest::Approx(9.2e8).epsilon(0.03));   // about 0.93 million km
    }

    TEST_CASE("eccentric orbit: the Hill radius uses a (1 - e)")
    {
        Sys s;
        s.add(1.0e20, 1.0e8, {0, 0, 0}, {0, 0, 0});
        const double a = 4.0e10, e = 0.5, m = 1.0e14;
        s.addOrbiter(0, m, 1.0e6, a, e);
        const auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);
        CHECK(radii[1].hill == doctest::Approx(a * (1.0 - e) * std::cbrt(m / (3.0e20))).epsilon(1e-9));
        CHECK(radii[1].soi == doctest::Approx(a * std::pow(m / 1.0e20, 0.4)).epsilon(1e-9));   // SOI does not take e
    }

    TEST_CASE("Roche limit: 2.44 R_p (rho_p / rho_m)^(1/3), centred on the parent")
    {
        Sys s;
        const double muP = 3.986e14, rP = 6.371e6, muM = 4.9e12, rM = 1.737e6;
        s.add(muP, rP, {1.0e9, 2.0e9, 0}, {0, 0, 0});
        s.addOrbiter(0, muM, rM, 3.844e8, 0.05);
        const auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);
        const double expect = 2.44 * rP * std::cbrt(density(muP, rP) / density(muM, rM));
        CHECK(radii[1].roche == doctest::Approx(expect).epsilon(1e-9));
        CHECK(radii[1].rocheCentre == 0);
        CHECK(radii[1].roche > 9.0e6);    // Earth/Moon fluid Roche limit is about 18,000 km for the Moon's density
        CHECK(radii[1].roche < 2.0e7);
        CHECK(radii[0].roche == 0.0);     // the primary has no parent, so no circle
        CHECK(radii[0].parent == -1);
    }

    TEST_CASE("a moon's parent is its planet, not the star")
    {
        Sys s;
        s.add(1.327e20, 6.96e8, {0, 0, 0}, {0, 0, 0});
        s.addOrbiter(0, 3.986e14, 6.371e6, 1.496e11, 0.0);
        s.addOrbiter(1, 4.9e12, 1.737e6, 3.844e8, 0.0);
        const auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);
        CHECK(radii[2].parent == 1);
        CHECK(radii[2].rocheCentre == 1);
        CHECK(radii[2].hill == doctest::Approx(3.844e8 * std::cbrt(4.9e12 / (3.0 * 3.986e14))).epsilon(1e-6));
        CHECK(radii[1].parent == 0);
    }

    TEST_CASE("unbound or lone bodies get no circles")
    {
        Sys s;
        s.add(1.0e20, 1.0e8, {0, 0, 0}, {0, 0, 0});
        s.add(1.0e14, 1.0e6, {1.0e10, 0, 0}, {0, 5.0e5, 0});    // far above escape speed
        auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);
        CHECK(radii[1].hill == 0.0);
        CHECK(radii[1].soi == 0.0);

        Sys one;
        one.add(1.0e20, 1.0e8, {0, 0, 0}, {0, 0, 0});
        radii = computeBodyRadii(one.r, one.v, one.mu, one.radius);
        REQUIRE(radii.size() == 1);
        CHECK(radii[0].hill == 0.0);
        CHECK(radii[0].parent == -1);
        CHECK(computeBodyRadii(std::vector<Vec3>(), std::vector<Vec3>(), std::vector<double>(), std::vector<double>()).empty());
    }

    TEST_CASE("appendCircles places Hill and SOI on the body and Roche on the parent, and honours the mask")
    {
        Sys s;
        s.add(1.327e20, 6.96e8, {0, 0, 0}, {0, 0, 0});
        s.addOrbiter(0, 3.986e14, 6.371e6, 1.496e11, 0.0);
        const auto radii = computeBodyRadii(s.r, s.v, s.mu, s.radius);

        RenderFrame f;
        f.bodies.resize(2);
        f.bodies[0].x = 10.0f; f.bodies[0].y = -20.0f;
        f.bodies[1].x = 1.5e11f; f.bodies[1].y = 7.0f;

        appendCircles(f, radii);
        REQUIRE(f.circles.size() == 3);                   // Hill + SOI + Roche of body 1; the star has none
        CHECK(f.circles[0].kind == CircleKind::Hill);
        CHECK(f.circles[0].x == 1.5e11f);
        CHECK(f.circles[1].kind == CircleKind::Soi);
        CHECK(f.circles[1].y == 7.0f);
        CHECK(f.circles[2].kind == CircleKind::Roche);
        CHECK(f.circles[2].x == 10.0f);                   // centred on the star
        CHECK(f.circles[2].y == -20.0f);

        f.clearObjects();
        f.bodies.resize(2);
        CircleSet only;
        only.hill = false; only.roche = false;
        appendCircles(f, radii, only);
        REQUIRE(f.circles.size() == 1);
        CHECK(f.circles[0].kind == CircleKind::Soi);
    }

    TEST_CASE("atmosphere bubbles and well sources come from the display bodies")
    {
        std::vector<DisplayBody> bodies(3);
        bodies[0].mu = 1.0e18; bodies[0].radius = 2.0e8;
        bodies[1].mu = 3.767e12; bodies[1].radius = 6.2e5;
        bodies[1].atmosphere = Atmosphere{1.22, 7000.0};
        bodies[1].colour = {0.28f, 0.48f, 0.78f};
        bodies[2].mu = 0.0; bodies[2].radius = 1.0;                      // massless: no well
        bodies[2].atmosphere = Atmosphere{0.0, 100.0};                   // rho0 below the cutoff: no bubble

        RenderFrame f;
        f.bodies.resize(3);
        f.bodies[1].x = 5.0f; f.bodies[1].y = 6.0f;

        appendWells(f, bodies);
        REQUIRE(f.wells.size() == 2);
        CHECK(f.wells[1].x == 5.0f);
        CHECK(f.wells[1].radius == doctest::Approx(6.2e5f));
        CHECK(f.wells[1].mu == doctest::Approx(3.767e12f).epsilon(1e-6));

        appendAtmospheres(f, bodies);
        REQUIRE(f.atmospheres.size() == 1);
        CHECK(f.atmospheres[0].y == 6.0f);
        CHECK(f.atmospheres[0].top == doctest::Approx(static_cast<float>(7000.0 * std::log(1.22 / 1e-6))).epsilon(1e-6));
        CHECK(f.atmospheres[0].scaleHeight == 7000.0f);
        CHECK(f.atmospheres[0].tint.b > f.atmospheres[0].tint.r);        // a blue world gets a blue sky
    }

    TEST_CASE("radii cache: recomputes at its cadence, and at once when the generation or the bodies change")
    {
        std::vector<DisplayBody> bodies(2);
        bodies[0].id = 1; bodies[0].mu = 1.327e20; bodies[0].radius = 6.96e8;
        bodies[1].id = 2; bodies[1].mu = 3.986e14; bodies[1].radius = 6.371e6;
        bodies[1].r = {1.496e11, 0.0, 0.0};
        bodies[1].v = {0.0, 29780.0, 0.0};

        RadiiCache cache;
        cache.minRealGap = 0.25;
        CHECK(cache.get(bodies, 1, 10.0).size() == 2);
        CHECK(cache.computations() == 1);
        cache.get(bodies, 1, 10.1);                       // inside the gap: cached
        CHECK(cache.computations() == 1);
        cache.get(bodies, 1, 10.3);                       // gap passed
        CHECK(cache.computations() == 2);
        cache.get(bodies, 2, 10.31);                      // a rebuilt simulation
        CHECK(cache.computations() == 3);
        bodies[1].id = 3;                                 // the set of bodies changed
        cache.get(bodies, 2, 10.32);
        CHECK(cache.computations() == 4);
        bodies.pop_back();
        CHECK(cache.get(bodies, 2, 10.33).size() == 1);
        CHECK(cache.computations() == 5);
        cache.get(bodies, 2, 1.0);                        // the clock went backwards
        CHECK(cache.computations() == 6);
    }
}
