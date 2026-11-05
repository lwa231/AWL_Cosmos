// [3d-core] spec: 3D Jacobi placement (src/core/placement3d.h). Skipped by default; run with
//   cosmos_tests --no-skip --test-case=*3d-core*placement*
#include <doctest.h>
#include <cmath>
#include <vector>
#include "core/kepler3d.h"
#include "core/placement.h"
#include "core/placement3d.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {

PlacementBody3D body(double mu, int parent, double a = 0.0, double e = 0.0, double i = 0.0, double Om = 0.0,
                     double w = 0.0, double M = 0.0)
{
    PlacementBody3D b;
    b.mu = mu;
    b.parent = parent;
    b.orbit = {a, e, i, Om, w, M};
    return b;
}

// Star, two inclined planets (one retrograde), a moon of the inner one on its own inclined orbit, and a moon of the moon.
std::vector<PlacementBody3D> inclinedSystem()
{
    return {
        body(1.327e20, -1),                                                      // 0 star
        body(3.986e14, 0, 1.496e11, 0.0167, 7.0 * kDeg, 0.3, 1.8, 0.4),          // 1 planet
        body(1.267e17, 0, 7.785e11, 0.048, 160.0 * kDeg, 1.75, 4.8, 2.0),        // 2 retrograde giant
        body(4.905e12, 1, 3.844e8, 0.055, 28.0 * kDeg, 2.2, 5.5, -1.0),          // 3 moon of 1
        body(1.0e8, 3, 2.0e7, 0.1, 80.0 * kDeg, 0.5, 0.2, 3.0),                  // 4 moonlet of 3
    };
}

Vec3 sumMu(const std::vector<PlacementBody3D>& in, const std::vector<Placed>& out, bool velocity)
{
    Vec3 s;
    for (std::size_t k = 0; k < in.size(); ++k) s += in[k].mu * (velocity ? out[k].v : out[k].r);
    return s;
}

} // namespace

TEST_CASE("[3d-core] placement: barycentre at the origin and zero total momentum, in all three axes" * doctest::skip())
{
    const auto in = inclinedSystem();
    const auto out = placeJacobi3D(in);
    REQUIRE(out.size() == in.size());
    double muTot = 0.0, rScale = 0.0, vScale = 0.0;
    for (std::size_t k = 0; k < in.size(); ++k) {
        muTot += in[k].mu;
        rScale += in[k].mu * norm(out[k].r);
        vScale += in[k].mu * norm(out[k].v);
    }
    const Vec3 R = sumMu(in, out, false);
    const Vec3 P = sumMu(in, out, true);
    CHECK(norm(R) / rScale < 1e-14);
    CHECK(norm(P) / vScale < 1e-14);
    // Really out of plane: the planets and moons have z and vz.
    CHECK(std::abs(out[1].r.z) > 1e9);
    CHECK(std::abs(out[3].v.z - out[1].v.z) > 10.0);
    (void)muTot;
}

TEST_CASE("[3d-core] placement: every Jacobi relative state has the requested 3D elements" * doctest::skip())
{
    const auto in = inclinedSystem();
    const auto out = placeJacobi3D(in);
    REQUIRE(out.size() == in.size());
    auto bary = [&](std::initializer_list<int> ids, bool vel) {
        Vec3 s;
        double m = 0.0;
        for (int k : ids) {
            s += in[k].mu * (vel ? out[k].v : out[k].r);
            m += in[k].mu;
        }
        return s / m;
    };
    auto muOf = [&](std::initializer_list<int> ids) {
        double m = 0.0;
        for (int k : ids) m += in[k].mu;
        return m;
    };
    struct Rel { Vec3 r, v; double mu; int body; };
    // Moonlet 4 about moon 3. Moon 3 (+4) about planet 1. Planet subsystem {1,3,4} about the star. The giant 2 (outer,
    // a larger) about the barycentre of the star plus the planet's subsystem {0,1,3,4}.
    const Rel cases[] = {
        {out[4].r - out[3].r, out[4].v - out[3].v, muOf({3, 4}), 4},
        {bary({3, 4}, false) - out[1].r, bary({3, 4}, true) - out[1].v, muOf({1, 3, 4}), 3},
        {bary({1, 3, 4}, false) - out[0].r, bary({1, 3, 4}, true) - out[0].v, muOf({0, 1, 3, 4}), 1},
        {out[2].r - bary({0, 1, 3, 4}, false), out[2].v - bary({0, 1, 3, 4}, true), muOf({0, 1, 2, 3, 4}), 2},
    };
    for (const Rel& c : cases) {
        const OrbitSpec3D& o = in[c.body].orbit;
        const Elements3D el = stateToElements3D(c.r, c.v, c.mu);
        CAPTURE(c.body);
        CHECK(near(el.a, o.a, 1e-10));
        CHECK(near(el.e, o.e, 1e-9));
        CHECK(std::abs(el.i - o.i) < 1e-10);
        CHECK(std::abs(angleDiff(el.Omega, o.Omega)) < 1e-9);
        CHECK(std::abs(angleDiff(el.omega, o.omega)) < 1e-8);
        CHECK(std::abs(angleDiff(el.M, o.M)) < 1e-8);
    }
}

TEST_CASE("[3d-core] placement: a planar 2D system reduces to placeJacobi, exactly in the plane" * doctest::skip())
{
    std::vector<PlacementBody> in2;
    auto add = [&](double mu, int parent, double a, double e, double w, double M, bool retro) {
        PlacementBody b;
        b.mu = mu;
        b.parent = parent;
        b.orbit = {a, e, w, M, retro};
        in2.push_back(b);
    };
    add(1.989e30 * 6.674e-11, -1, 0, 0, 0, 0, false);
    add(3.986e14, 0, 1.496e11, 0.0167, 1.8, 0.4, false);
    add(1.267e17, 0, 7.785e11, 0.048, 4.8, 2.0, true);
    add(4.905e12, 1, 3.844e8, 0.055, 5.5, -1.0, true);
    add(1.0e8, 3, 2.0e7, 0.1, 0.2, 3.0, false);

    const auto ref = placeJacobi(in2);
    const auto in3 = toPlacement3D(in2);
    REQUIRE(in3.size() == in2.size());
    for (std::size_t k = 0; k < in2.size(); ++k) {
        CHECK(in3[k].mu == in2[k].mu);
        CHECK(in3[k].parent == in2[k].parent);
        if (k == 0) continue;
        CHECK(in3[k].orbit.i == (in2[k].orbit.retrograde ? kPi : 0.0));
        CHECK(in3[k].orbit.Omega == 0.0);
        CHECK(std::abs(angleDiff(in3[k].orbit.omega, in2[k].orbit.retrograde ? -in2[k].orbit.omega : in2[k].orbit.omega)) < 1e-14);
        CHECK(in3[k].orbit.a == in2[k].orbit.a);
        CHECK(in3[k].orbit.e == in2[k].orbit.e);
        CHECK(in3[k].orbit.M == in2[k].orbit.M);
    }
    const auto out = placeJacobi3D(in3);
    REQUIRE(out.size() == ref.size());
    for (std::size_t k = 0; k < out.size(); ++k) {
        CAPTURE(k);
        CHECK(out[k].r.z == 0.0);
        CHECK(out[k].v.z == 0.0);
        // relative to the largest scale in play, so the star (near the origin) is held to the same absolute standard
        CHECK(norm(out[k].r - ref[k].r) <= 1e-12 * 7.785e11);
        CHECK(norm(out[k].v - ref[k].v) <= 1e-12 * 3.0e4);
    }
}

TEST_CASE("[3d-core] placement: two bodies, the orbit plane is the requested one" * doctest::skip())
{
    const double i = 40.0 * kDeg, Om = 1.2;
    const std::vector<PlacementBody3D> in = {body(1.0e20, -1), body(1.0e19, 0, 1.0e10, 0.2, i, Om, 0.7, 1.0)};
    const auto out = placeJacobi3D(in);
    REQUIRE(out.size() == 2);
    const Vec3 h = cross(out[1].r - out[0].r, out[1].v - out[0].v);
    const Vec3 n{std::sin(i) * std::sin(Om), -std::sin(i) * std::cos(Om), std::cos(i)};
    CHECK(relErr(h / norm(h), n) < 1e-13);
    // Two-body barycentre split: r_0 = -mu_1 / (mu_0 + mu_1) * d
    const Vec3 d = out[1].r - out[0].r;
    CHECK(relErr(out[0].r, d * (-1.0e19 / 1.1e20)) < 1e-13);
}

TEST_CASE("[3d-core] placement: invalid input gives an empty result (same rules as placeJacobi)" * doctest::skip())
{
    CHECK(placeJacobi3D({}).empty());
    CHECK(placeJacobi3D({body(1.0, -1), body(1.0, -1)}).empty());              // two roots
    CHECK(placeJacobi3D({body(1.0, 0, 1.0)}).empty());                          // no root (self parent)
    CHECK(placeJacobi3D({body(1.0, -1), body(1.0, 5, 1.0)}).empty());           // parent out of range
    CHECK(placeJacobi3D({body(1.0, -1), body(1.0, 2, 1.0), body(1.0, 1, 2.0)}).empty()); // cycle, unreachable
    CHECK(placeJacobi3D({body(1.0, -1)}).size() == 1);
}
