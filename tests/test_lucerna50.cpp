#include <doctest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>
#include "core/diagnostics.h"
#include "core/gravity.h"
#include "core/integrators.h"
#include "core/radii.h"
#include "core/scene.h"
#include "test_scene_util.h"

using namespace cosmos;
using scene_test::Lucerna;

// Plan 14 "Lucerna 50-year": the section 13 system (with the gear-marked values) loaded from scenes/lucerna.json,
// resolved with Jacobi placement and integrated for 50 Domus years with Yoshida-4 at dt = 500 s.
//
// Runtime (981,244 steps of 14 bodies): about 2.4 s in Release and 28 s in Debug on the development machine, inside the
// 60 s budget, so it runs in every configuration. If a slower machine ever pushes Debug over budget, give the test case
// `* doctest::skip(true)` under `#ifndef NDEBUG` so that only Release/RelWithDebInfo run it, and run it by hand in
// Debug with   ./build/cosmos_tests.exe -ts="lucerna50" --no-skip

namespace {

struct MoonTrack {
    std::string name;
    std::size_t moon = 0;      // store indices
    std::size_t planet = 0;
    double muPair = 0.0;
    double radii = 0.0;        // sum of the two radii
    double hill = 0.0;         // Hill radius of the planet about the star at its semi-major axis
    bool retrograde = false;
    double eps0 = 0.0;         // two-body specific energy at the start
    double maxSepOverHill = 0.0;
    double minSepOverRadii = std::numeric_limits<double>::infinity();
    double maxEpsRatio = -std::numeric_limits<double>::infinity();    // eps / |eps0|; bound while < 0
    double minEpsRatio = std::numeric_limits<double>::infinity();
    bool senseKept = true;     // angular momentum about the planet never changed sign
};

} // namespace

TEST_SUITE("lucerna50")
{
    TEST_CASE("Lucerna 50-year: moons bound, Exul retrograde, Gelu inside 0.4 Hill, energy error < 1e-9")
    {
        Lucerna L;
        Bodies& b = L.res.bodies;
        const Scene& sc = L.scene;

        const double domusYear = 2.0 * 3.141592653589793238462643383279502884 *
                                 std::sqrt(std::pow(L.body("Domus").orbit->a, 3) / L.body("Lucerna").mu);
        const double dt = 500.0;
        const double duration = 50.0 * domusYear;   // about 4.9e8 s
        const long long steps = static_cast<long long>(std::ceil(duration / dt));

        // Every moon against its planet (scene parent)
        std::vector<MoonTrack> moons;
        const double muStar = L.body("Lucerna").mu;
        for (std::size_t i = 0; i < sc.bodies.size(); ++i) {
            const SceneBody& m = sc.bodies[i];
            if (m.kind != BodyKind::Moon) continue;
            const std::size_t p = static_cast<std::size_t>(findSceneBody(sc, m.orbit->parent));
            const SceneBody& pl = sc.bodies[p];
            MoonTrack t;
            t.name = m.name;
            t.moon = L.storeIndex(m.name);
            t.planet = L.storeIndex(pl.name);
            t.muPair = m.mu + pl.mu;
            t.radii = m.radius + pl.radius;
            t.hill = hillRadius(pl.orbit->a, 0.0, pl.mu, muStar);
            t.retrograde = m.orbit->retrograde;
            moons.push_back(t);
        }
        REQUIRE(moons.size() == 7);

        auto relative = [&](const MoonTrack& t, Vec3& r, Vec3& v) {
            const BodyState& s = b.state;
            r = Vec3{s.x[t.moon] - s.x[t.planet], s.y[t.moon] - s.y[t.planet], s.z[t.moon] - s.z[t.planet]};
            v = Vec3{s.vx[t.moon] - s.vx[t.planet], s.vy[t.moon] - s.vy[t.planet], s.vz[t.moon] - s.vz[t.planet]};
        };
        auto observe = [&]() {
            for (MoonTrack& t : moons) {
                Vec3 r, v;
                relative(t, r, v);
                const double d = norm(r);
                const double eps = 0.5 * norm2(v) - t.muPair / d;
                t.maxSepOverHill = std::max(t.maxSepOverHill, d / t.hill);
                t.minSepOverRadii = std::min(t.minSepOverRadii, d / t.radii);
                t.maxEpsRatio = std::max(t.maxEpsRatio, eps / std::fabs(t.eps0));
                t.minEpsRatio = std::min(t.minEpsRatio, eps / std::fabs(t.eps0));
                const double lz = r.x * v.y - r.y * v.x;
                if (t.retrograde ? !(lz < 0.0) : !(lz > 0.0)) t.senseKept = false;
            }
        };
        for (MoonTrack& t : moons) {
            Vec3 r, v;
            relative(t, r, v);
            t.eps0 = 0.5 * norm2(v) - t.muPair / norm(r);
            REQUIRE(t.eps0 < 0.0);
        }

        const Conserved c0 = conserved(b);
        double worstEnergy = 0.0;
        double worstL = 0.0;
        auto checkConserved = [&]() {
            const Conserved c = conserved(b);
            worstEnergy = std::max(worstEnergy, std::fabs((c.energyG - c0.energyG) / c0.energyG));
            worstL = std::max(worstL, std::fabs((c.angularMomentumG.z - c0.angularMomentumG.z) / c0.angularMomentumG.z));
        };

        Accel a;
        computeAccelerations(b.state, b.mu, a);
        observe();
        const auto wall0 = std::chrono::steady_clock::now();
        for (long long k = 1; k <= steps; ++k) {
            stepYoshida4(b.state, b.mu, a, dt);
            observe();
            if (k % 1000 == 0) checkConserved();
        }
        checkConserved();
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();

        std::printf("[lucerna50] %lld steps (%.4g s = %.2f Domus years) in %.1f s wall; max energy error %.3g; max |dLz|/Lz %.3g\n",
                    steps, steps * dt, steps * dt / domusYear, wall, worstEnergy, worstL);
        for (const MoonTrack& t : moons)
            std::printf("[lucerna50] %-8s max sep/Hill %.4f  min sep/(R1+R2) %.3f  eps/|eps0| in [%.4f, %.4f]\n",
                        t.name.c_str(), t.maxSepOverHill, t.minSepOverRadii, t.minEpsRatio, t.maxEpsRatio);

        CHECK(worstEnergy < 1e-9);       // measured 4.3e-10; the plan measured <= 3.9e-10 on the pre-gear values
        CHECK(worstL < 1e-12);           // measured 6.6e-14; the plan: < 3e-13

        for (const MoonTrack& t : moons) {
            INFO(t.name);
            CHECK(t.maxEpsRatio < -0.5);               // bound at every step (measured worst -0.96; unbound is >= 0)
            CHECK(t.maxSepOverHill < 0.4);             // inside 0.4 of the planet's Hill radius (measured worst 0.33, Gelu)
            CHECK(t.senseKept);                        // never flipped its sense of revolution (Exul stays retrograde)
            // Measured: Maxima-Ignis 3.60 closest, everything else >= 7.65 (plan: 3.6 and >= 7.6)
            CHECK(t.minSepOverRadii > (t.name == "Ignis" ? 3.5 : 7.5));
        }
        for (const MoonTrack& t : moons) {
            if (t.name == "Exul") {
                CHECK(t.retrograde);
                CHECK(t.maxSepOverHill < 0.15);        // measured 0.097; the plan's limit for a retrograde moon is 0.6
            }
        }
    }
}
