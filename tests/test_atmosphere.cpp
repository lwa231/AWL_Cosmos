#include <doctest.h>
#include <cmath>
#include "core/atmosphere.h"
#include "core/constants.h"
#include "test_scene_util.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Atmospheres and exact drag (plan 5.5, 5.6, 13, 14 "Drag").

TEST_SUITE("atmosphere")
{
    TEST_CASE("density is rho0 exp(-h/H); at and below the surface it is rho0")
    {
        const Atmosphere a{1.22, 7000.0};
        CHECK(atmosphereDensity(a, 0.0) == 1.22);
        CHECK(atmosphereDensity(a, -50.0) == 1.22);
        CHECK(atmosphereDensity(a, 7000.0) == doctest::Approx(1.22 * std::exp(-1.0)).epsilon(1e-15));
        CHECK(atmosphereDensity(a, 35000.0) == doctest::Approx(1.22 * std::exp(-5.0)).epsilon(1e-15));
        CHECK(atmosphereDensity(Atmosphere{0.0, 7000.0}, 100.0) == 0.0);
        CHECK(atmosphereDensity(Atmosphere{1.0, 0.0}, 100.0) == 0.0);
    }

    TEST_CASE("top is H ln(rho0 / 1e-6): density there is exactly 1e-6 and drag stops at it")
    {
        const Atmosphere a{1.22, 7000.0};
        const double top = atmosphereTop(a);
        CHECK(top == doctest::Approx(7000.0 * std::log(1.22e6)).epsilon(1e-15));
        CHECK(atmosphereDensity(a, top) == doctest::Approx(kAtmosphereCutoffDensity).epsilon(1e-12));
        CHECK(dragDensity(a, top * (1.0 - 1e-9)) > kAtmosphereCutoffDensity);
        CHECK(dragDensity(a, top) == 0.0);
        CHECK(dragDensity(a, top + 1.0) == 0.0);
        // Thinner than the cutoff already at the surface: no atmosphere
        CHECK(atmosphereTop(Atmosphere{5e-7, 7000.0}) == 0.0);
        CHECK(dragDensity(Atmosphere{5e-7, 7000.0}, 0.0) == 0.0);
    }

    TEST_CASE("Lucerna: atmosphere tops and surface pressures match the plan's section 13 numbers")
    {
        scene_test::Lucerna L;
        struct Row { const char* name; double topKm; double pKPa; };
        // Plan 13: tops Domus ~98, Velum ~138, Rubor ~117, Maxima ~772, Aura ~89 km. Pressures: Domus 83.7 kPa, Velum 4.99 bar,
        // Rubor 2.0 kPa, Maxima 1 bar, Aura 9.8 kPa. The plan's g column is rounded to 3 digits, so P0 is compared to 1%.
        const Row rows[] = {{"Domus", 98.0, 83.7}, {"Velum", 138.0, 499.0}, {"Rubor", 117.0, 2.0}, {"Maxima", 772.0, 100.0}, {"Aura", 89.0, 9.8}};
        for (const Row& row : rows) {
            CAPTURE(row.name);
            const std::size_t i = L.storeIndex(row.name);
            REQUIRE(L.res.bodies.atmosphere[i].has_value());
            const Atmosphere& a = *L.res.bodies.atmosphere[i];
            CHECK(atmosphereTop(a) / 1000.0 == doctest::Approx(row.topKm).epsilon(0.005)); // half a percent: the plan says "~"
            const double p = surfacePressure(a, L.res.bodies.mu[i], L.res.bodies.radius[i]) / 1000.0;
            CHECK(p == doctest::Approx(row.pKPa).epsilon(0.02));
        }
    }

    TEST_CASE("Domus: g = 9.80 and P0 = 83.7 kPa")
    {
        scene_test::Lucerna L;
        const std::size_t i = L.storeIndex("Domus");
        const double g = surfaceGravity(L.res.bodies.mu[i], L.res.bodies.radius[i]);
        CHECK(g == doctest::Approx(9.80).epsilon(5e-4));
        const double p0 = surfacePressure(*L.res.bodies.atmosphere[i], L.res.bodies.mu[i], L.res.bodies.radius[i]);
        CHECK(p0 == doctest::Approx(83.7e3).epsilon(1e-3));
        // Exact: 1.22 * (3.767e12 / 6.2e5^2) * 7000
        CHECK(p0 == doctest::Approx(1.22 * 3.767e12 / (6.2e5 * 6.2e5) * 7000.0).epsilon(1e-14));
        CHECK(atmosphereTop(*L.res.bodies.atmosphere[i]) / 1000.0 == doctest::Approx(98.1).epsilon(2e-3));
    }

    TEST_CASE("air rotates with the body: v_air = v_body + omega z x (r - r_body)")
    {
        const Vec3 rb{1e9, -2e9, 0.0}, vb{100.0, 50.0, 0.0};
        // Counter-clockwise spin: at +x of the body the air moves toward +y
        Vec3 v = airVelocity(vb, 7.292e-5, rb, rb + Vec3{6.2e5, 0.0, 0.0});
        CHECK(v.x == doctest::Approx(100.0).epsilon(1e-15));
        CHECK(v.y == doctest::Approx(50.0 + 7.292e-5 * 6.2e5).epsilon(1e-15));
        // At +y it moves toward -x
        v = airVelocity(vb, 7.292e-5, rb, rb + Vec3{0.0, 6.2e5, 0.0});
        CHECK(v.x == doctest::Approx(100.0 - 7.292e-5 * 6.2e5).epsilon(1e-15));
        CHECK(v.y == doctest::Approx(50.0).epsilon(1e-15));
        // Retrograde (clockwise) spin flips the sign; no spin leaves the body velocity
        v = airVelocity(vb, -1e-4, rb, rb + Vec3{1000.0, 0.0, 0.0});
        CHECK(v.y == doctest::Approx(50.0 - 0.1).epsilon(1e-15));
        v = airVelocity(vb, 0.0, rb, rb + Vec3{1000.0, 1000.0, 0.0});
        CHECK(v.x == 100.0);
        CHECK(v.y == 50.0);
    }

    TEST_CASE("exact drag: v(t) = v0 / (1 + k v0 t), to 1e-12, in one step and in many")
    {
        const double rho = 1.0e-3, cdA = 2.0, m = 5000.0;
        const double k = dragRate(rho, cdA, m);
        CHECK(k == doctest::Approx(2e-7).epsilon(1e-15));
        const Vec3 v0{3000.0, 4000.0, 0.0}; // speed 5000
        const double T = 1500.0;
        const double expect = 5000.0 / (1.0 + k * 5000.0 * T);

        Vec3 one = applyExactDrag(v0, Vec3{}, k, T);
        CHECK(norm(one) == doctest::Approx(expect).epsilon(1e-12));
        // The direction is unchanged
        CHECK(one.x / norm(one) == doctest::Approx(0.6).epsilon(1e-14));

        Vec3 many = v0;
        for (int i = 0; i < 1000; ++i) many = applyExactDrag(many, Vec3{}, k, T / 1000.0);
        CHECK(norm(many) == doctest::Approx(expect).epsilon(1e-12));
        // Two halves equal the whole (the solution is a semigroup)
        Vec3 halves = applyExactDrag(applyExactDrag(v0, Vec3{}, k, 0.5 * T), Vec3{}, k, 0.5 * T);
        CHECK(norm(halves) == doctest::Approx(expect).epsilon(1e-12));
    }

    TEST_CASE("exact drag solves dv/dt = -k |v| v: derivative check, and stability for huge steps")
    {
        const double k = 3e-6;
        const Vec3 v0{800.0, -600.0, 0.0};
        const double dt = 1e-3;
        const Vec3 v1 = applyExactDrag(v0, Vec3{}, k, dt);
        const Vec3 dv = (v1 - v0) / dt;
        CHECK(dv.x == doctest::Approx(-k * 1000.0 * 800.0).epsilon(1e-3)); // first-order finite difference
        CHECK(dv.y == doctest::Approx(-k * 1000.0 * -600.0).epsilon(1e-3));
        // A huge step in thick air stays finite, never reverses the velocity and tends to rest
        const Vec3 big = applyExactDrag(v0, Vec3{}, 1.0, 1e12);
        CHECK(std::isfinite(big.x));
        CHECK(big.x * v0.x >= 0.0);
        CHECK(norm(big) < 1e-6);
        // Zero drag changes nothing, bit for bit
        const Vec3 same = applyExactDrag(v0, Vec3{}, 0.0, 100.0);
        CHECK(same.x == v0.x);
        CHECK(same.y == v0.y);
    }

    TEST_CASE("exact drag acts on the velocity relative to the moving air")
    {
        const Vec3 vAir{0.0, 465.0, 0.0}; // air moving with the rotating planet
        const double k = 1e-6;
        // A shuttle moving with the air feels nothing
        const Vec3 same = applyExactDrag(vAir, vAir, k, 100.0);
        CHECK(same.x == vAir.x);
        CHECK(same.y == vAir.y);
        // Otherwise the relative velocity decays and v tends to the air velocity
        const Vec3 v0{7000.0, 465.0, 0.0};
        const double dt = 20.0;
        const Vec3 v1 = applyExactDrag(v0, vAir, k, dt);
        const double rel0 = 7000.0;
        CHECK(v1.x == doctest::Approx(rel0 / (1.0 + k * rel0 * dt)).epsilon(1e-14));
        CHECK(v1.y == doctest::Approx(465.0).epsilon(1e-14));
        const Vec3 late = applyExactDrag(v0, vAir, 1.0, 1e9);
        CHECK(late.y == doctest::Approx(465.0).epsilon(1e-9));
        CHECK(std::fabs(late.x) < 1e-3);
    }

    TEST_CASE("drag inside a propagation at (almost) constant density matches v0 / (1 + k v0 t)")
    {
        // A massless body with a huge scale height makes the density constant over the path: rho = rho0 to 1e-12.
        Bodies b;
        BodyDesc d = bodyDesc(0.0, 1.0);
        d.atmosphere = Atmosphere{1.0e-3, 1.0e18};
        b.add(d);
        LinearEphemeris eph(b);
        ControllerSet cs;
        ShuttleEnvironment env(b, eph, cs);
        REQUIRE(env.atmosphereTop[0] > 1e18); // 1e18 ln(1e3)

        Shuttles s;
        ShuttleDesc sd = shuttleDesc(1, {1.0e6, 0.0, 0.0}, {3000.0, 4000.0, 0.0});
        sd.dryMass = 5000.0;
        sd.fuelMass = 0.0;
        sd.cdA = 2.0;
        REQUIRE(s.add(sd));
        std::vector<Event> ev;
        ShuttleStats stats;
        const double T = 100.0;
        propagateShuttle(s, 0, 0.0, T, env, ev, nullptr, &stats);

        const double k = dragRate(1.0e-3, 2.0, 5000.0);
        const double expect = 5000.0 / (1.0 + k * 5000.0 * T);
        CHECK(norm(shuttleVel(s, 0)) == doctest::Approx(expect).epsilon(1e-12));
        // direction unchanged
        CHECK(s.state.vx[0] / norm(shuttleVel(s, 0)) == doctest::Approx(0.6).epsilon(1e-12));
        CHECK(stats.acceptedSteps > 1000); // the step is held to 0.02 s of dynamical time (1 s without a gravitating body)
        CHECK(ev.empty());
    }
}
