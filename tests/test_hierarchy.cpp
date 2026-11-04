#include <doctest.h>
#include <cmath>
#include <random>
#include <set>
#include <vector>
#include "core/hierarchy.h"
#include "core/placement.h"
#include "lucerna_data.h"

using namespace cosmos;

namespace {
const double kPi = 3.141592653589793238462643383279502884;

enum { Lucerna, Favilla, Velum, Domus, Custos, Gelu, Rubor, Exul, Maxima, Ignis, Unda, Aura, Hiems, Sodalis };

struct Scene {
    std::vector<Vec3> r, v;
    std::vector<double> mu;
};

Scene sceneFrom(const std::vector<PlacementBody>& bodies)
{
    const auto placed = placeJacobi(bodies);
    Scene s;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        s.r.push_back(placed[i].r);
        s.v.push_back(placed[i].v);
        s.mu.push_back(bodies[i].mu);
    }
    return s;
}

void leavesOf(const Hierarchy& h, int node, std::set<int>& out)
{
    if (h.nodes[node].childA == -1) {
        out.insert(node);
        return;
    }
    leavesOf(h, h.nodes[node].childA, out);
    leavesOf(h, h.nodes[node].childB, out);
}

// A planet orbits the barycentre of the star and inner planets: that node contains the star, not the planet or its moons
bool planetOrbitsStarGroup(const Hierarchy& h, int planet, std::set<int> own)
{
    own.insert(planet);
    std::set<int> leaves;
    leavesOf(h, h.bodyParent[planet], leaves);
    if (!leaves.count(Lucerna)) return false;
    for (int i : own) if (leaves.count(i)) return false;
    return true;
}

double uni(std::mt19937_64& g)
{
    return static_cast<double>(g() >> 11) * (1.0 / 9007199254740992.0);
}
}

TEST_SUITE("hierarchy")
{
    TEST_CASE("empty and single-body systems")
    {
        Hierarchy e = buildHierarchy({}, {}, {});
        CHECK(e.root == -1);
        CHECK(e.nodes.empty());
        CHECK(shuttleParent(e, {1, 2, 3}) == -1);

        Hierarchy one = buildHierarchy({{0, 0, 0}}, {{0, 0, 0}}, {1.0});
        CHECK(one.root == 0);
        CHECK(one.bodyParent[0] == -1);
        CHECK(one.nodes.size() == 1);
    }

    TEST_CASE("star and planet: one barycentre node, planet orbits the star")
    {
        Hierarchy h = buildHierarchy({{0, 0, 0}, {10, 0, 0}}, {{0, 0, 0}, {0, 1, 0}}, {100.0, 1.0});
        REQUIRE(h.nodes.size() == 3);
        CHECK(h.root == 2);
        CHECK(h.nodes[2].childA == 0);
        CHECK(h.nodes[2].childB == 1);
        CHECK(h.nodes[2].mu == 101.0);
        CHECK(h.nodes[2].r.x == doctest::Approx(10.0 / 101.0).epsilon(1e-14).scale(0));
        CHECK(h.nodes[2].v.y == doctest::Approx(1.0 / 101.0).epsilon(1e-14).scale(0));
        CHECK(h.bodyParent[0] == -1);
        CHECK(h.bodyParent[1] == 0);
        CHECK(h.nodes[0].parent == 2);
    }

    TEST_CASE("hill radius: lighter node uses d*cbrt(mu/3mu_sibling), heavier inherits, root is infinite")
    {
        Hierarchy h = buildHierarchy({{0, 0, 0}, {10, 0, 0}}, {{}, {}}, {81.0, 3.0});
        CHECK(hillRadius(h, 1) == doctest::Approx(10.0 * std::cbrt(3.0 / (3.0 * 81.0))).epsilon(1e-14).scale(0));
        CHECK(std::isinf(hillRadius(h, 0)));
        CHECK(std::isinf(hillRadius(h, h.root)));
    }

    TEST_CASE("equal-mass triple: the distant body orbits the close pair's barycentre")
    {
        Hierarchy h = buildHierarchy({{0, 0, 0}, {1, 0, 0}, {10, 0, 0}}, {{}, {}, {}}, {1.0, 1.0, 1.0});
        REQUIRE(h.nodes.size() == 5);
        const int pair = h.nodes[0].parent;
        CHECK(h.nodes[1].parent == pair);
        CHECK(pair >= 3);
        CHECK(h.bodyParent[2] == pair);
        CHECK(h.bodyParent[1] == 0);
        CHECK(h.bodyParent[0] == -1);
    }

    TEST_CASE("a nearby but unbound pair is not treated as parent and child")
    {
        // Two light bodies 20 apart, 1000 from a heavy star: their Hill sphere is only about 9
        Hierarchy h = buildHierarchy({{0, 0, 0}, {1000, 0, 0}, {1020, 0, 0}}, {{}, {}, {}}, {1e6, 1.0, 1.0});
        const int pair = h.nodes[1].parent;
        CHECK_FALSE(h.nodes[pair].bound);
        CHECK(h.bodyParent[1] == 0);
        CHECK(h.bodyParent[2] == 0);
    }

    TEST_CASE("Lucerna: moons orbit their planets, planets orbit the star group")
    {
        const Scene s = sceneFrom(lucerna_data::lucerna());
        const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);

        CHECK(h.bodyParent[Lucerna] == -1);
        CHECK(planetOrbitsStarGroup(h, Favilla, {}));
        CHECK(planetOrbitsStarGroup(h, Velum, {}));
        CHECK(planetOrbitsStarGroup(h, Domus, {Custos, Gelu}));
        CHECK(planetOrbitsStarGroup(h, Rubor, {Exul}));
        CHECK(planetOrbitsStarGroup(h, Maxima, {Ignis, Unda, Aura}));
        CHECK(planetOrbitsStarGroup(h, Hiems, {Sodalis}));
        CHECK(h.bodyParent[Favilla] == Lucerna);

        CHECK(h.bodyParent[Custos] == Domus);
        CHECK(h.bodyParent[Exul] == Rubor);
        CHECK(h.bodyParent[Ignis] == Maxima);
        CHECK(h.bodyParent[Sodalis] == Hiems);

        // Outer moons orbit the barycentre of the planet plus its inner moons
        std::set<int> gelu, unda, aura;
        leavesOf(h, h.bodyParent[Gelu], gelu);
        leavesOf(h, h.bodyParent[Unda], unda);
        leavesOf(h, h.bodyParent[Aura], aura);
        CHECK(gelu == std::set<int>{Domus, Custos});
        CHECK(unda == std::set<int>{Maxima, Ignis});
        CHECK(aura == std::set<int>{Maxima, Ignis, Unda});
    }

    TEST_CASE("Lucerna: the same relations hold for 300 random orbital phases")
    {
        std::mt19937_64 g(777);
        for (int trial = 0; trial < 300; ++trial) {
            auto bodies = lucerna_data::lucerna();
            for (std::size_t i = 1; i < bodies.size(); ++i) bodies[i].orbit.M = (2.0 * uni(g) - 1.0) * kPi;
            const Scene s = sceneFrom(bodies);
            const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);

            CAPTURE(trial);
            CHECK(planetOrbitsStarGroup(h, Favilla, {}));
            CHECK(planetOrbitsStarGroup(h, Velum, {}));
            CHECK(planetOrbitsStarGroup(h, Domus, {Custos, Gelu}));
            CHECK(planetOrbitsStarGroup(h, Rubor, {Exul}));
            CHECK(planetOrbitsStarGroup(h, Maxima, {Ignis, Unda, Aura}));
            CHECK(planetOrbitsStarGroup(h, Hiems, {Sodalis}));
            CHECK(h.bodyParent[Custos] == Domus);
            CHECK(h.bodyParent[Exul] == Rubor);
            CHECK(h.bodyParent[Ignis] == Maxima);
            CHECK(h.bodyParent[Sodalis] == Hiems);

            std::set<int> gelu, unda, aura;
            leavesOf(h, h.bodyParent[Gelu], gelu);
            leavesOf(h, h.bodyParent[Unda], unda);
            leavesOf(h, h.bodyParent[Aura], aura);
            CHECK(gelu == std::set<int>{Domus, Custos});
            CHECK(unda == std::set<int>{Maxima, Ignis});
            CHECK(aura == std::set<int>{Maxima, Ignis, Unda});
        }
    }

    TEST_CASE("shuttle parent: deepest Hill sphere containing it, else the system barycentre")
    {
        const Scene s = sceneFrom(lucerna_data::lucerna());
        const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);

        const Vec3 away = s.r[Domus] - s.r[Custos];
        const Vec3 nearDomus = s.r[Domus] + 3.0e5 * (away / norm(away));
        CHECK(shuttleParent(h, nearDomus) == Domus);

        const Vec3 nearCustos = s.r[Custos] + Vec3{4.0e5, 0, 0};
        CHECK(shuttleParent(h, nearCustos) == Custos);

        const Vec3 deep = {-3.0e12, -3.0e12, 0};
        CHECK(shuttleParent(h, deep) == h.root);
    }

    TEST_CASE("shuttle parent: the smallest Hill sphere wins, and a body wins over a barycentre of equal radius")
    {
        const Scene s = sceneFrom(lucerna_data::lucerna());
        const Hierarchy h = buildHierarchy(s.r, s.v, s.mu);
        const int n = static_cast<int>(s.mu.size());

        // Gelu is deeper in the tree than Domus's barycentre but its sphere (2.3e6 m) is the smaller one; Domus inherits the
        // 1.4e8 m sphere of the planet system, which also contains the point: the old rule (deepest node) took Domus
        // for a shuttle 2.1e5 m from Gelu.
        CHECK(hillRadius(h, Gelu) < 3.0e6);
        CHECK(hillRadius(h, Domus) > 1.0e8);
        for (double dist : {2.1e5, 6.0e5, 1.5e6}) {
            const Vec3 p = s.r[Gelu] + Vec3{dist, 0.0, 0.0};
            INFO("distance from Gelu " << dist);
            CHECK(shuttleParent(h, p) == Gelu);
        }
        // Low orbits of the planet: the body, never the Domus + Custos barycentre (same radius, inherited)
        for (double dist : {1.0e6, 2.0e6, 3.0e6}) {
            const Vec3 p = s.r[Domus] + Vec3{0.0, dist, 0.0};
            INFO("distance from Domus " << dist);
            const int parent = shuttleParent(h, p);
            CHECK(parent == Domus);
            CHECK(parent < n);
        }
        // The table version chooses the same parent for every point
        HillTable table;
        buildHillTable(h, table);
        for (double dist : {2.1e5, 6.0e5, 1.5e6, 1.0e7, 1.0e8, 3.0e9}) {
            for (int who = 0; who < n; ++who) {
                const Vec3 p = s.r[who] + Vec3{dist, 0.3 * dist, 0.0};
                CHECK(shuttleParent(h, table, p) == shuttleParent(h, p));
            }
        }
        // The same ordering on the comparison itself
        const ParentCandidate small{1.0e6, false, 3, 5.0e5}, big{1.0e8, true, 9, 1.0e5};
        CHECK(betterParent(small, big));
        CHECK(!betterParent(big, small));
        const ParentCandidate body{1.0e8, true, 4, 9.0e7}, bary{1.0e8, false, 5, 1.0e5};
        CHECK(betterParent(body, bary));   // equal radius: the body, whatever the depth or distance
        CHECK(!betterParent(bary, body));
        const ParentCandidate deep{1.0e8, false, 6, 9.0e7}, shallow{1.0e8, false, 5, 1.0e5};
        CHECK(betterParent(deep, shallow));   // then the deeper one
        const ParentCandidate near{1.0e8, false, 5, 1.0e5}, far{1.0e8, false, 5, 2.0e5};
        CHECK(betterParent(near, far));       // then the nearer one
    }
}
