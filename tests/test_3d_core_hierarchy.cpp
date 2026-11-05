// [3d-core] spec: the hierarchy and Hill radius in 3D. These functions (hierarchy.h) already use the full 3D
// distance, so there is nothing new to declare: the spec pins that behaviour out of plane, so a 3D rewrite cannot
// lose it. Skipped by default; run with
//   cosmos_tests --no-skip --test-case=*3d-core*hierarchy*
#include <doctest.h>
#include <cmath>
#include <vector>
#include "core/hierarchy.h"
#include "core/radii.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {

const double kMuStar = 1.0e20, kMuPlanet = 1.0e16, kMuMoon = 1.0e12;
const double kAPlanet = 1.0e10, kAMoon = 1.0e7;

struct Sys {
    std::vector<Vec3> r, v;
    std::vector<double> mu;
};

// Star (0), planet (1) and its moon (2). moonOffset is the moon's position relative to the planet.
Sys system(const Vec3& moonOffset)
{
    Sys s;
    const Vec3 rp{kAPlanet, 0.0, 0.0};
    s.r = {{0.0, 0.0, 0.0}, rp, rp + moonOffset};
    s.v = {{0.0, 0.0, 0.0}, {0.0, std::sqrt(kMuStar / kAPlanet), 0.0},
           {0.0, std::sqrt(kMuStar / kAPlanet) + std::sqrt(kMuPlanet / kAMoon), 0.0}};
    s.mu = {kMuStar, kMuPlanet, kMuMoon};
    return s;
}

Sys rotated(const Sys& in)
{
    Sys s = in;
    for (Vec3& p : s.r) p = rotQ(p);
    for (Vec3& p : s.v) p = rotQ(p);
    return s;
}

} // namespace

TEST_CASE("[3d-core] hierarchy: in-plane Hill radii match the closed form (the 3D radius equals the 2D one)" *
          doctest::skip())
{
    const Sys s = system({kAMoon, 0.0, 0.0});
    const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);
    REQUIRE(h.root >= 0);
    CHECK(h.bodyParent[1] == 0);
    CHECK(h.bodyParent[2] == 1);
    // The planet+moon node is the lighter child of the root: Hill radius d cbrt(mu / (3 mu_star)), as radii.h has it.
    const int pm = h.nodes[1].parent;
    REQUIRE(pm >= 3);
    const double d = norm(h.nodes[pm].r - s.r[0]);
    CHECK(hillRadius(h, pm) == doctest::Approx(hillRadius(d, 0.0, kMuPlanet + kMuMoon, kMuStar)).epsilon(1e-14));
    CHECK(hillRadius(h, 1) == hillRadius(h, pm)); // heavier child inherits
    CHECK(hillRadius(h, 2) == doctest::Approx(hillRadius(kAMoon, 0.0, kMuMoon, kMuPlanet)).epsilon(1e-14));
}

TEST_CASE("[3d-core] hierarchy: the tree and Hill radii do not change when the whole system is rotated out of plane" *
          doctest::skip())
{
    const Sys flat = system({0.0, kAMoon, 0.0});
    const Sys tilt = rotated(flat);
    const Hierarchy a = buildHierarchy(flat.r, flat.v, flat.mu);
    const Hierarchy b = buildHierarchy(tilt.r, tilt.v, tilt.mu);
    REQUIRE(a.nodes.size() == b.nodes.size());
    CHECK(a.root == b.root);
    CHECK(a.bodyParent == b.bodyParent);
    for (std::size_t i = 0; i < a.nodes.size(); ++i) {
        CAPTURE(i);
        CHECK(a.nodes[i].childA == b.nodes[i].childA);
        CHECK(a.nodes[i].childB == b.nodes[i].childB);
        CHECK(a.nodes[i].bound == b.nodes[i].bound);
        const double ha = hillRadius(a, static_cast<int>(i)), hb = hillRadius(b, static_cast<int>(i));
        if (std::isfinite(ha)) CHECK(hb == doctest::Approx(ha).epsilon(1e-12));
        else CHECK_FALSE(std::isfinite(hb));
        CHECK(norm(b.nodes[i].r - rotQ(a.nodes[i].r)) < 1e-12 * kAPlanet);
    }
}

TEST_CASE("[3d-core] hierarchy: a moon straight above its planet (same x, y) is found by its 3D distance" *
          doctest::skip())
{
    const Sys s = system({0.0, 0.0, kAMoon});
    const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);
    CHECK(h.bodyParent[2] == 1);
    CHECK(hillRadius(h, 2) == doctest::Approx(kAMoon * std::cbrt(kMuMoon / (3.0 * kMuPlanet))).epsilon(1e-12));
}

TEST_CASE("[3d-core] hierarchy: shuttle parent by 3D distance, not by its x-y projection" * doctest::skip())
{
    const Sys s = system({kAMoon, 0.0, 0.0});
    const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);
    HillTable table;
    buildHillTable(h, table);
    const double hillPlanet = hillRadius(h, 1);
    REQUIRE(hillPlanet > 1.0e8);

    // Above the planet, inside its Hill sphere: the planet.
    const Vec3 inside = s.r[1] + Vec3{0.0, 0.0, 0.5 * hillPlanet};
    CHECK(shuttleParent(h, inside) == 1);
    CHECK(shuttleParent(h, table, inside) == 1);
    // Same x and y, but higher than the Hill radius: not the planet (the system root).
    const Vec3 above = s.r[1] + Vec3{0.0, 0.0, 2.0 * hillPlanet};
    CHECK(shuttleParent(h, above) == h.root);
    CHECK(shuttleParent(h, table, above) == h.root);
    // Just above the moon: the moon.
    const Vec3 nearMoon = s.r[2] + Vec3{0.0, 0.0, 0.5 * hillRadius(h, 2)};
    CHECK(shuttleParent(h, nearMoon) == 2);
    CHECK(shuttleParent(h, table, nearMoon) == 2);
}
