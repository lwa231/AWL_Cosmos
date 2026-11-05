// [3d-core] spec: 3D impact prediction (src/core/impact3d.h). Skipped by default; run with
//   cosmos_tests --no-skip --test-case=*3d-core*impact*
#include <doctest.h>
#include <cmath>
#include <random>
#include "core/impact.h"
#include "core/impact3d.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {

const double kMu = 3.986004418e14;
const double kR = 6.371e6;

void checkSame(const ImpactPrediction& p, const ImpactPrediction& ref, double tol)
{
    REQUIRE(p.kind == ref.kind);
    if (ref.kind != ImpactKind::Impact) return;
    CHECK(near(p.t, ref.t, tol));
    CHECK(std::abs(p.nuHit - ref.nuHit) < tol);
    CHECK(near(p.speedHit, ref.speedHit, tol));
}

} // namespace

TEST_CASE("[3d-core] impact: in-plane states reduce to the 2D predictParentImpact" * doctest::skip())
{
    std::mt19937_64 g(5);
    int impacts = 0, n = 0;
    for (int k = 0; k < 500; ++k) {
        const double rm = kR * (1.05 + 20.0 * uni(g));
        const double th = 2.0 * kPi * uni(g);
        const double speed = std::sqrt(kMu / rm) * (0.05 + 1.6 * uni(g));
        const double dir = 2.0 * kPi * uni(g);
        const Vec3 r{rm * std::cos(th), rm * std::sin(th), 0.0};
        const Vec3 v{speed * std::cos(dir), speed * std::sin(dir), 0.0};
        const ImpactPrediction ref = predictParentImpact(r, v, kMu, kR);
        const ImpactPrediction p = predictParentImpact3D(r, v, kMu, kR);
        CAPTURE(k);
        checkSame(p, ref, 1e-11);
        if (ref.hits()) {
            CHECK(p.point.z == 0.0);
            CHECK(relErr(p.point, ref.point) < 1e-11);
            ++impacts;
        }
        ++n;
    }
    CHECK(impacts > 50);
    // Inside / InsideBound / Invalid too
    CHECK(predictParentImpact3D({kR * 0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, kMu, kR).kind == ImpactKind::InsideBound);
    CHECK(predictParentImpact3D({0.0, 0.0, kR * 0.9}, {0.0, 0.0, 2.0e4}, kMu, kR).kind == ImpactKind::Inside);
    CHECK(predictParentImpact3D({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, kMu, kR).kind == ImpactKind::Invalid);
    CHECK(predictParentImpact3D({1.0e7, 0.0, 0.0}, {0.0, 1.0, 0.0}, -1.0, kR).kind == ImpactKind::Invalid);
    CHECK(predictParentImpact3D({1.0e7, 0.0, NAN}, {0.0, 1.0, 0.0}, kMu, kR).kind == ImpactKind::Invalid);
}

TEST_CASE("[3d-core] impact: rotating the whole state rotates the crossing point and changes nothing else" *
          doctest::skip())
{
    std::mt19937_64 g(77);
    int impacts = 0;
    for (int k = 0; k < 300; ++k) {
        const double rm = kR * (1.05 + 20.0 * uni(g));
        const double th = 2.0 * kPi * uni(g);
        const double speed = std::sqrt(kMu / rm) * (0.05 + 1.6 * uni(g));
        const double dir = 2.0 * kPi * uni(g);
        const Vec3 r{rm * std::cos(th), rm * std::sin(th), 0.0};
        const Vec3 v{speed * std::cos(dir), speed * std::sin(dir), 0.0};
        const ImpactPrediction flat = predictParentImpact(r, v, kMu, kR);
        const ImpactPrediction tilted = predictParentImpact3D(rotQ(r), rotQ(v), kMu, kR);
        CAPTURE(k);
        checkSame(tilted, flat, 1e-10);
        if (flat.hits()) {
            CHECK(relErr(tilted.point, rotQ(flat.point)) < 1e-10);
            CHECK(std::abs(norm(tilted.point) - kR) < 1e-9 * kR);
            ++impacts;
        }
    }
    CHECK(impacts > 30);
}

TEST_CASE("[3d-core] impact: radial fall along z, the 2D predictor cannot see it" * doctest::skip())
{
    // Dropped from rest at 2 R straight above the north pole. 2D sees r = (0, 0) and reports Invalid.
    const Vec3 r{0.0, 0.0, 2.0 * kR};
    const Vec3 v{0.0, 0.0, 0.0};
    CHECK(predictParentImpact(r, v, kMu, kR).kind == ImpactKind::Invalid);
    const ImpactPrediction p = predictParentImpact3D(r, v, kMu, kR);
    REQUIRE(p.hits());
    // Radial free fall from rest at r0 to r: t = sqrt(r0^3 / (2 mu)) (sqrt(x(1-x)) + acos(sqrt(x))), x = r / r0.
    const double r0 = 2.0 * kR, x = 0.5;
    const double t = std::sqrt(r0 * r0 * r0 / (2.0 * kMu)) * (std::sqrt(x * (1.0 - x)) + std::acos(std::sqrt(x)));
    CHECK(p.t == doctest::Approx(t).epsilon(1e-10));
    CHECK(p.speedHit == doctest::Approx(std::sqrt(2.0 * kMu * (1.0 / kR - 1.0 / r0))).epsilon(1e-12));
    CHECK(relErr(p.point, {0.0, 0.0, kR}) < 1e-12);
}

TEST_CASE("[3d-core] impact: an inclined flyby that hits in projection but misses in 3D" * doctest::skip())
{
    // Coming in along +x from far away, offset 3 R in z. The x-y projection is a head-on radial fall (2D: Impact),
    // but the real periapsis is well above the surface.
    const double V = 3000.0;
    const Vec3 r{-200.0 * kR, 0.0, 3.0 * kR};
    const Vec3 v{V, 0.0, 0.0};
    CHECK(predictParentImpact(r, v, kMu, kR).hits());
    const ImpactPrediction p = predictParentImpact3D(r, v, kMu, kR);
    CHECK(p.kind == ImpactKind::None);

    // Offset 0.5 R in z instead: it does hit, on the side facing +z, in the x-z plane.
    const Vec3 r2{-200.0 * kR, 0.0, 0.5 * kR};
    const ImpactPrediction hit = predictParentImpact3D(r2, v, kMu, kR);
    REQUIRE(hit.hits());
    CHECK(std::abs(norm(hit.point) - kR) < 1e-9 * kR);
    CHECK(std::abs(hit.point.y) < 1e-6 * kR);
    CHECK(hit.point.z > 0.0);
    CHECK(hit.point.x < 0.0); // gravity bends it in, but it is still on the near side
    // Point lies in the orbit plane
    const Vec3 h = cross(r2, v);
    CHECK(std::abs(dot(hit.point, h)) < 1e-12 * norm(hit.point) * norm(h));
    // Energy at the crossing (vis-viva) and a time a little less than the straight-line estimate
    CHECK(hit.speedHit == doctest::Approx(std::sqrt(V * V + 2.0 * kMu / kR - 2.0 * kMu / norm(r2))).epsilon(1e-12));
    CHECK(hit.t < 200.0 * kR / V);
    CHECK(hit.t > 0.8 * 200.0 * kR / V);
}

TEST_CASE("[3d-core] impact: shuttle and pair wrappers use the full 3D relative state" * doctest::skip())
{
    const Vec3 rp{1.0e11, -2.0e10, 3.0e9}, vp{1.0e3, 2.9e4, -50.0};
    const Vec3 rel = rotQ({-30.0 * kR, 0.0, 0.2 * kR});
    const Vec3 vrel = rotQ({5000.0, 0.0, 0.0});
    const ImpactPrediction direct = predictParentImpact3D(rel, vrel, kMu, triggerRadius(kR, std::nullopt));
    REQUIRE(direct.hits());

    const ImpactPrediction s = predictShuttleImpact3D(rp + rel, vp + vrel, rp, vp, kMu, kR, std::nullopt);
    checkSame(s, direct, 1e-8); // absolute positions at 1e11 m cost some digits in the subtraction
    Atmosphere air;
    air.rho0 = 1.2;
    air.scaleHeight = 8500.0;
    const ImpactPrediction sa = predictShuttleImpact3D(rp + rel, vp + vrel, rp, vp, kMu, kR, air);
    REQUIRE(sa.hits());
    CHECK(sa.t < s.t);
    CHECK(norm(sa.point) == doctest::Approx(triggerRadius(kR, air)).epsilon(1e-9));

    const double muB = 4.9e12, rB = 1.737e6;
    const ImpactPrediction pr = predictPairImpact3D(rp, vp, kMu - muB, kR - rB, rp + rel, vp + vrel, muB, rB);
    checkSame(pr, direct, 1e-8);
    CHECK(relErr(pr.point, direct.point) < 1e-8);
}
