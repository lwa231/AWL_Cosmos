#include <doctest.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include "app/presets.h"
#include "core/constants.h"
#include "core/scene.h"
#include "core/simulation.h"
#include "data/horizons_j2000_plus_1yr.h"
#include "test_scene_util.h"

using namespace cosmos;

// Plan 14 "Real solar system (M8)" and plan 11.2: the Real solar system preset (scenes/solar_system.json, built from JPL
// Horizons barycentric vectors at J2000 = JD 2451545.0 TDB) is integrated with the app's IAS15 Simulation for 365.25 days
// and compared with the Horizons vectors at JD 2451910.25 TDB (tests/data/horizons_j2000_plus_1yr.h, stored, never fetched
// at test time; see tests/data/README.md for the source).
//
// What the model leaves out, and so what the errors are: the 11 bodies are point masses with Newtonian gravity. There is
// no 1PN relativity (Mercury), no Ceres/Pallas/Vesta and the other asteroids, no Charon/Triton/giant-planet moons (the
// planets use their body GM, not the system GM, and are compared at the body centre, as Horizons reports them), and no
// J2 or tides. Pluto is the extreme case: the Pluto body circles the Pluto-Charon barycentre (about 2,100 km away) every
// 6.4 days, which a single point mass cannot reproduce, so its error is of that size by construction.

namespace {

struct BodyError {
    std::string name;
    double posKm = 0.0;     // |r_sim - r_horizons|
    double velMs = 0.0;     // |v_sim - v_horizons|, m/s
};

} // namespace

TEST_SUITE("real solar system")
{
    TEST_CASE("the preset is registered, available and loads through the app's preset loader")
    {
        const app::PresetInfo* p = app::findPreset("solar_system");
        REQUIRE(p != nullptr);
        CHECK(p->name == "Real solar system");
        CHECK(p->file == "solar_system.json");
        CHECK(app::presetAvailable(*p, std::string(COSMOS_SOURCE_DIR) + "/scenes"));

        Scene s;
        std::string err;
        REQUIRE_MESSAGE(app::loadPreset("solar_system", std::string(COSMOS_SOURCE_DIR) + "/scenes", s, err), err);
        CHECK(s.name == "Real solar system");
        REQUIRE(s.bodies.size() == 11);
        CHECK(s.shuttles.empty());
        CHECK(s.epoch_s == 0.0);

        const char* names[11] = {"Sun", "Mercury", "Venus", "Earth", "Moon", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"};
        const BodyKind kinds[11] = {BodyKind::Star, BodyKind::Planet, BodyKind::Planet, BodyKind::Planet, BodyKind::Moon,
                                    BodyKind::Planet, BodyKind::GasGiant, BodyKind::GasGiant, BodyKind::GasGiant,
                                    BodyKind::GasGiant, BodyKind::DwarfPlanet};
        for (std::size_t i = 0; i < 11; ++i) {
            CHECK(s.bodies[i].name == names[i]);
            CHECK(s.bodies[i].kind == kinds[i]);
            CHECK(s.bodies[i].kindLocked);
            CHECK_FALSE(s.bodies[i].orbit.has_value());   // explicit Horizons states
            CHECK(s.bodies[i].mu > 0.0);
            CHECK(s.bodies[i].radius > 0.0);
        }
        // Real z is kept: Pluto is far out of the ecliptic (about 0.88e12 m), Earth is not exactly in it.
        CHECK(s.bodies[10].position.z > 8.0e11);
        CHECK(s.bodies[3].position.z != 0.0);
        // Spot checks against the Horizons headers
        CHECK(s.bodies[0].mu == doctest::Approx(1.32712440041939e20).epsilon(1e-12));
        CHECK(s.bodies[3].mu == doctest::Approx(3.98600435436e14).epsilon(1e-12));
        CHECK(s.bodies[4].radius == doctest::Approx(1.73753e6).epsilon(1e-12));

        // Explicit states are taken verbatim (no barycentre shift) and the store holds all 11 bodies
        ResolvedScene res;
        REQUIRE_MESSAGE(resolveScene(s, res, err), err);
        REQUIRE(res.bodies.size() == 11);
        for (std::size_t i = 0; i < 11; ++i) {
            const auto idx = res.bodies.find(res.bodyIds[i]);
            REQUIRE(idx.has_value());
            CHECK(res.bodies.state.x[*idx] == s.bodies[i].position.x);
            CHECK(res.bodies.state.vz[*idx] == s.bodies[i].velocity.z);
        }
        // Sanity: Earth is 1 AU from the Sun at J2000 (perihelion season, 0.983 AU)
        const Vec3 d = s.bodies[3].position - s.bodies[0].position;
        CHECK(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) / constants::AU == doctest::Approx(0.9833).epsilon(1e-3));
    }

    TEST_CASE("Real solar system (M8): 1 year of IAS15 against JPL Horizons")
    {
        Scene scene;
        std::string err;
        REQUIRE_MESSAGE(loadSceneFile(scene_test::sceneFile("solar_system.json"), scene, err), err);
        ResolvedScene res;
        REQUIRE_MESSAGE(resolveScene(scene, res, err), err);

        SimulationOptions opts;
        opts.threads = 1;
        opts.startTime = scene.epoch_s;
        Simulation sim(res, scene.settings, opts);
        REQUIRE(sim.bodies().size() == 11);

        const double tEnd = (horizons::kPlusOneYearJD - horizons::kEpochJD) * 86400.0;   // 31,557,600 s
        const auto wall0 = std::chrono::steady_clock::now();
        REQUIRE(sim.advanceTo(tEnd));
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
        REQUIRE(sim.time() >= tEnd);
        REQUIRE(sim.hasBodyStep());
        REQUIRE(sim.bodyStepStart() <= tEnd);

        // The step that ends at or after tEnd is not clipped to it: read the state at exactly tEnd from its dense output.
        BodyError err_[11];
        for (std::size_t i = 0; i < 11; ++i) {
            const horizons::StateKm& ref = horizons::kPlusOneYear[i];
            REQUIRE(scene.bodies[i].name == ref.name);
            Vec3 r, v;
            REQUIRE(sim.bodyPositionAt(res.bodyIds[i], tEnd, r));
            REQUIRE(sim.bodyVelocityAt(res.bodyIds[i], tEnd, v));
            const double dx = r.x - ref.x * 1e3, dy = r.y - ref.y * 1e3, dz = r.z - ref.z * 1e3;
            const double dvx = v.x - ref.vx * 1e3, dvy = v.y - ref.vy * 1e3, dvz = v.z - ref.vz * 1e3;
            err_[i].name = ref.name;
            err_[i].posKm = std::sqrt(dx * dx + dy * dy + dz * dz) / 1e3;
            err_[i].velMs = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
        }

        std::printf("\n[real solar system] J2000 + 365.25 d, IAS15 (%ld steps, %.3f s wall)\n", sim.stats().bodySteps, wall);
        std::printf("  %-8s %14s %14s\n", "body", "position (km)", "velocity (m/s)");
        for (const BodyError& e : err_) std::printf("  %-8s %14.3f %14.6f\n", e.name.c_str(), e.posKm, e.velMs);

        CHECK_FALSE(sim.failed());
        CHECK(sim.stats().merges == 0);
        // Plan 14: Earth within about 1,000 km
        CHECK(err_[3].posKm < 1000.0);
        // Mercury: the documented drift of about 24 km/yr from the missing relativity (1PN comes later)
        CHECK(err_[1].posKm < 200.0);
        // The rest of the inner system (Sun, Venus, Moon, Mars) stays within a few hundred km as well
        for (const std::size_t i : {0u, 2u, 4u, 5u}) CHECK_MESSAGE(err_[i].posKm < 300.0, err_[i].name);
        // The outer bodies are compared at the body centre, which circles the system barycentre with its moons (Io to
        // Callisto, Titan, Triton, Charon). Their Horizons velocities carry that wobble (about 1 m/s for Jupiter to
        // Neptune, 27 m/s for Pluto), which a point mass at the body centre integrates as a constant drift: 1 m/s over a
        // year is 31,600 km. These bounds only guard against regressions; they are not an accuracy claim. Using the system
        // barycentres (Horizons 5 to 9) with the system GMs would remove most of it.
        CHECK(err_[6].posKm < 5.0e3);     // Jupiter, measured 2.7e3
        CHECK(err_[7].posKm < 6.0e4);     // Saturn, measured 4.1e4
        CHECK(err_[8].posKm < 1.5e4);     // Uranus, measured 8.1e3
        CHECK(err_[9].posKm < 4.5e4);     // Neptune, measured 2.9e4
        CHECK(err_[10].posKm < 1.2e6);    // Pluto, measured 7.6e5
    }
}
