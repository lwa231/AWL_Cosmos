#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "core/kepler.h"
#include "core/placement.h"
#include "lucerna_data.h"

using namespace cosmos;
using lucerna_data::root;
using lucerna_data::child;
using lucerna_data::lucerna;

namespace {
const double kPi = 3.141592653589793238462643383279502884;
const double kDeg = kPi / 180.0;

double wrapPi(double a)
{
    double t = std::fmod(a + kPi, 2.0 * kPi);
    if (t <= 0.0) t += 2.0 * kPi;
    return t - kPi;
}

void collect(int b, const std::vector<PlacementBody>& bodies, std::vector<int>& out)
{
    out.push_back(b);
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i)
        if (bodies[i].parent == b) collect(i, bodies, out);
}

struct Group {
    double mu = 0.0;
    Vec3 r;
    Vec3 v;
};

Group barycentre(const std::vector<int>& idx, const std::vector<PlacementBody>& bodies, const std::vector<Placed>& placed)
{
    Group g;
    for (int i : idx) {
        g.mu += bodies[i].mu;
        g.r += bodies[i].mu * placed[i].r;
        g.v += bodies[i].mu * placed[i].v;
    }
    g.r = g.r / g.mu;
    g.v = g.v / g.mu;
    return g;
}

// Elements of body k's subsystem about the barycentre of its parent plus its inner siblings' subsystems
Elements recovered(int k, const std::vector<PlacementBody>& bodies, const std::vector<Placed>& placed)
{
    const int p = bodies[k].parent;
    std::vector<int> inner{p};
    for (int s = 0; s < static_cast<int>(bodies.size()); ++s) {
        if (bodies[s].parent != p || s == k) continue;
        const bool before = bodies[s].orbit.a < bodies[k].orbit.a ||
                            (bodies[s].orbit.a == bodies[k].orbit.a && s < k);
        if (before) collect(s, bodies, inner);
    }
    std::vector<int> own;
    collect(k, bodies, own);

    const Group x = barycentre(inner, bodies, placed);
    const Group y = barycentre(own, bodies, placed);
    return stateToElements(y.r - x.r, y.v - x.v, x.mu + y.mu);
}
}

TEST_SUITE("placement")
{
    TEST_CASE("a single body sits at rest at the origin")
    {
        auto out = placeJacobi({root(5.0)});
        REQUIRE(out.size() == 1);
        CHECK(norm(out[0].r) == 0.0);
        CHECK(norm(out[0].v) == 0.0);
    }

    TEST_CASE("two bodies: relative state is the Kepler state, barycentre at rest at the origin")
    {
        const double mu1 = 1000.0, mu2 = 10.0;
        auto out = placeJacobi({root(mu1), child(mu2, 0, 5.0, 0.3, 40, 100)});
        REQUIRE(out.size() == 2);

        Elements el;
        el.a = 5.0;
        el.e = 0.3;
        el.omega = 40 * kDeg;
        el.M = 100 * kDeg;
        const RelState rel = elementsToState(el, mu1 + mu2);

        CHECK(norm((out[1].r - out[0].r) - rel.r) < 1e-12);
        CHECK(norm((out[1].v - out[0].v) - rel.v) < 1e-12);
        CHECK(norm(mu1 * out[0].r + mu2 * out[1].r) < 1e-10);
        CHECK(norm(mu1 * out[0].v + mu2 * out[1].v) < 1e-10);
    }

    TEST_CASE("input order of siblings does not change the result")
    {
        auto a = placeJacobi({root(100.0), child(1.0, 0, 3.0, 0.1, 10, 20), child(2.0, 0, 8.0, 0.05, 30, 40)});
        auto b = placeJacobi({root(100.0), child(2.0, 0, 8.0, 0.05, 30, 40), child(1.0, 0, 3.0, 0.1, 10, 20)});
        REQUIRE(a.size() == 3);
        REQUIRE(b.size() == 3);
        CHECK(norm(a[0].r - b[0].r) < 1e-12);
        CHECK(norm(a[1].r - b[2].r) < 1e-12);
        CHECK(norm(a[2].v - b[1].v) < 1e-12);
    }

    TEST_CASE("Lucerna: whole system is barycentric with zero momentum")
    {
        const auto bodies = lucerna();
        const auto placed = placeJacobi(bodies);
        REQUIRE(placed.size() == bodies.size());

        Vec3 pos, mom;
        double muSum = 0.0, scaleP = 0.0, scaleR = 0.0;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            pos += bodies[i].mu * placed[i].r;
            mom += bodies[i].mu * placed[i].v;
            muSum += bodies[i].mu;
            scaleP += bodies[i].mu * norm(placed[i].v);
            scaleR += bodies[i].mu * norm(placed[i].r);
        }
        CHECK(norm(pos) <= 1e-12 * scaleR);
        CHECK(norm(mom) <= 1e-12 * scaleP);
        CHECK(muSum > 0.0);
    }

    TEST_CASE("Lucerna: every body's orbit is recovered as specified")
    {
        const auto bodies = lucerna();
        const auto placed = placeJacobi(bodies);
        REQUIRE(placed.size() == bodies.size());

        for (int k = 1; k < static_cast<int>(bodies.size()); ++k) {
            CAPTURE(k);
            const Elements el = recovered(k, bodies, placed);
            const OrbitSpec& o = bodies[k].orbit;
            CHECK(el.a == doctest::Approx(o.a).epsilon(1e-9).scale(0));
            CHECK(std::abs(el.e - o.e) < 1e-8);
            CHECK(el.retrograde == o.retrograde);

            const double sgn = o.retrograde ? -1.0 : 1.0;
            CHECK(std::abs(wrapPi((el.omega + sgn * el.M) - (o.omega + sgn * o.M))) < 1e-6);
            if (o.e > 1e-3) {
                CHECK(std::abs(wrapPi(el.omega - o.omega)) < 1e-6);
            }
        }
    }

    TEST_CASE("Lucerna: Jacobi moon periods (plan section 13)")
    {
        const auto bodies = lucerna();
        const auto placed = placeJacobi(bodies);
        REQUIRE(placed.size() == bodies.size());

        auto periodHours = [&](int k) {
            const Elements el = recovered(k, bodies, placed);
            std::vector<int> inner{bodies[k].parent};
            for (int s = 0; s < static_cast<int>(bodies.size()); ++s)
                if (bodies[s].parent == bodies[k].parent && s != k && bodies[s].orbit.a < bodies[k].orbit.a)
                    collect(s, bodies, inner);
            std::vector<int> own;
            collect(k, bodies, own);
            double mu = 0.0;
            for (int i : inner) mu += bodies[i].mu;
            for (int i : own) mu += bodies[i].mu;
            return 2.0 * kPi * std::sqrt(el.a * el.a * el.a / mu) / 3600.0;
        };

        CHECK(periodHours(4) == doctest::Approx(18.72).epsilon(1e-3).scale(0));    // Custos
        CHECK(periodHours(5) == doctest::Approx(269.7).epsilon(1e-3).scale(0));    // Gelu
        CHECK(periodHours(7) == doctest::Approx(77.47).epsilon(1e-3).scale(0));    // Exul
        CHECK(periodHours(10) == doctest::Approx(41.84).epsilon(1e-3).scale(0));   // Unda
        CHECK(periodHours(13) == doctest::Approx(30.84).epsilon(1e-3).scale(0));   // Sodalis
    }

    TEST_CASE("invalid input gives an empty result")
    {
        CHECK(placeJacobi({}).empty());
        CHECK(placeJacobi({child(1.0, 1, 1, 0, 0, 0), child(1.0, 0, 1, 0, 0, 0)}).empty());          // no root
        CHECK(placeJacobi({root(1.0), root(2.0)}).empty());                                          // two roots
        CHECK(placeJacobi({root(1.0), child(1.0, 7, 1, 0, 0, 0)}).empty());                           // bad parent
        CHECK(placeJacobi({root(1.0), child(1.0, 2, 1, 0, 0, 0), child(1.0, 1, 1, 0, 0, 0)}).empty()); // cycle
        CHECK(placeJacobi({root(1.0), child(1.0, 1, 1, 0, 0, 0)}).empty());                           // own parent
    }
}
