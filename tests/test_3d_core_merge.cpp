// [3d-core] spec: vector spin and 3D angular momentum in merges (src/core/merge3d.h). Skipped by default; run with
//   cosmos_tests --no-skip --test-case=*3d-core*merge*
#include <doctest.h>
#include <cmath>
#include "core/bodies.h"
#include "core/merge.h"
#include "core/merge3d.h"
#include "core/radii.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {

SpinBody3D sb(double mu, double R, Vec3 r, Vec3 v, Vec3 spin, double k = 0.4)
{
    SpinBody3D b;
    b.mu = mu;
    b.radius = R;
    b.inertiaFactor = k;
    b.r = r;
    b.v = v;
    b.spin = spin;
    return b;
}

double volumeRadius(const SpinBody3D& a, const SpinBody3D& b)
{
    return std::cbrt(a.radius * a.radius * a.radius + b.radius * b.radius * b.radius);
}

// Total L about the origin after the merge: what the merged body carries, plus what was logged as lost.
Vec3 lAfter(const Merge3D& m)
{
    const Vec3 orbital = m.mu * cross(m.r, m.v);
    const Vec3 spin = m.inertiaFactor * m.mu * m.radius * m.radius * m.spin;
    return orbital + spin + m.lostAngularMomentumG;
}

// An off-plane grazing collision in dimensionless units: breakup spin of the result is about 30 rad/s, far above the
// spin it gets, so nothing is clamped.
SpinBody3D bigOne() { return sb(1.0, 0.1, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.3, -0.2, 1.0}); }
SpinBody3D smallOne() { return sb(0.2, 0.05, {0.08, 0.03, 0.09}, {-0.4, 0.1, -0.3}, {0.0, 2.0, -1.0}, 0.33); }

} // namespace

TEST_CASE("[3d-core] merge: spin angular momentum and total angular momentum of one body" * doctest::skip())
{
    const SpinBody3D b = sb(2.0, 3.0, {1.0, 2.0, 3.0}, {-1.0, 0.5, 0.25}, {0.1, 0.2, -0.3}, 0.35);
    const Vec3 S = spinAngularMomentumG(b);
    const double I = 0.35 * 2.0 * 9.0;
    CHECK(relErr(S, {I * 0.1, I * 0.2, I * -0.3}) < 1e-15);
    const Vec3 L = angularMomentumG(b);
    CHECK(relErr(L, 2.0 * cross(b.r, b.v) + S) < 1e-15);
}

TEST_CASE("[3d-core] merge: off-plane collision conserves mass, momentum, centre of mass and 3D angular momentum" *
          doctest::skip())
{
    const SpinBody3D a = bigOne(), b = smallOne();
    const double R = volumeRadius(a, b);
    const Merge3D m = mergeSpin3D(a, b, R);

    CHECK(m.mu == doctest::Approx(1.2).epsilon(1e-15));
    CHECK(m.radius == R);
    CHECK(m.inertiaFactor == 0.4); // the survivor's
    CHECK(relErr(m.mu * m.v, a.mu * a.v + b.mu * b.v) < 1e-15);
    CHECK(relErr(m.mu * m.r, a.mu * a.r + b.mu * b.r) < 1e-15);

    // Total L about the origin, and about an arbitrary other point (the choice of point must not matter).
    const Vec3 lBefore = angularMomentumG(a) + angularMomentumG(b);
    CHECK(relErr(lAfter(m), lBefore) < 1e-14);
    const Vec3 c{5.0, -3.0, 2.0};
    const Vec3 lBeforeC = a.mu * cross(a.r - c, a.v) + spinAngularMomentumG(a) + b.mu * cross(b.r - c, b.v) +
                          spinAngularMomentumG(b);
    const Vec3 lAfterC = m.mu * cross(m.r - c, m.v) + m.inertiaFactor * m.mu * R * R * m.spin + m.lostAngularMomentumG;
    CHECK(relErr(lAfterC, lBeforeC) < 1e-13);

    // Nothing lost: the off-plane orbital part is now in the spin vector, which tilts away from z.
    CHECK_FALSE(m.spinClamped);
    CHECK(m.lostAngularMomentumG.x == 0.0);
    CHECK(m.lostAngularMomentumG.y == 0.0);
    CHECK(m.lostAngularMomentumG.z == 0.0);
    CHECK(std::abs(m.spin.x) > 0.1);
    CHECK(std::abs(m.spin.y) > 0.1);
    CHECK(relErr(m.spin, m.omegaRaw) < 1e-15);

    // S_G is the spin part of the total about the new centre of mass.
    const Vec3 S = spinAngularMomentumG(a) + spinAngularMomentumG(b) + a.mu * cross(a.r - m.r, a.v - m.v) +
                   b.mu * cross(b.r - m.r, b.v - m.v);
    CHECK(relErr(m.spinAngularMomentumG, S) < 1e-14);
    CHECK(relErr(m.spin * (0.4 * 1.2 * R * R), S) < 1e-14);
    CHECK(m.omegaBreak == doctest::Approx(breakupSpin(1.2, R)).epsilon(1e-15));
}

TEST_CASE("[3d-core] merge: a spin above breakup is clamped along its own axis and the excess logged" * doctest::skip())
{
    // Same geometry, but the small body arrives fast enough that the merged spin would pass sqrt(mu / R^3).
    const SpinBody3D a = bigOne();
    SpinBody3D b = smallOne();
    b.v = {-40.0, 25.0, -30.0};
    const double R = volumeRadius(a, b);
    const Merge3D m = mergeSpin3D(a, b, R);
    REQUIRE(m.spinClamped);
    CHECK(norm(m.omegaRaw) > m.omegaBreak);
    CHECK(norm(m.spin) == doctest::Approx(m.omegaBreak).epsilon(1e-14));
    CHECK(relErr(m.spin / norm(m.spin), m.omegaRaw / norm(m.omegaRaw)) < 1e-14);
    const double I = 0.4 * 1.2 * R * R;
    CHECK(relErr(m.lostAngularMomentumG, m.spinAngularMomentumG - I * m.spin) < 1e-13);
    // The loss is parallel to the spin (only the length was cut)
    CHECK(norm(cross(m.lostAngularMomentumG, m.spin)) < 1e-12 * norm(m.lostAngularMomentumG) * norm(m.spin));
    const Vec3 lBefore = angularMomentumG(a) + angularMomentumG(b);
    CHECK(relErr(lAfter(m), lBefore) < 1e-14);
}

TEST_CASE("[3d-core] merge: point masses carry no spin, everything is logged as lost" * doctest::skip())
{
    const SpinBody3D a = sb(1.0, 0.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {});
    const SpinBody3D b = sb(0.5, 0.0, {0.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, {});
    const Merge3D m = mergeSpin3D(a, b, 0.0);
    CHECK(m.spin.x == 0.0);
    CHECK(m.spin.y == 0.0);
    CHECK(m.spin.z == 0.0);
    CHECK(relErr(m.lostAngularMomentumG, m.spinAngularMomentumG) < 1e-15);
    CHECK(norm(m.lostAngularMomentumG) > 0.0);
    CHECK(relErr(lAfter(m), angularMomentumG(a) + angularMomentumG(b)) < 1e-15);

    // Two massless bodies: the plain mean, no NaN.
    const Merge3D z = mergeSpin3D(sb(0.0, 1.0, {0, 0, 0}, {0, 0, 0}, {}), sb(0.0, 1.0, {2, 2, 2}, {1, 1, 1}, {}), 1.0);
    CHECK(relErr(z.r, {1.0, 1.0, 1.0}) < 1e-15);
    CHECK(relErr(z.v, {0.5, 0.5, 0.5}) < 1e-15);
    CHECK(std::isfinite(z.spin.x));
}

TEST_CASE("[3d-core] merge: an in-plane merge reduces to the 2D mergeBodies" * doctest::skip())
{
    for (double speed : {0.5, 60.0}) { // the second one clamps
        CAPTURE(speed);
        Bodies bodies;
        BodyDesc d1;
        d1.name = "big";
        d1.mu = 1.0;
        d1.radius = 0.1;
        d1.spinRate = 1.0;
        BodyDesc d2;
        d2.name = "small";
        d2.mu = 0.2;
        d2.radius = 0.05;
        d2.inertiaFactor = 0.33;
        d2.spinRate = -2.0;
        d2.position = {0.08, 0.06, 0.0};
        d2.velocity = {-speed, 0.3 * speed, 0.0};
        const BodyId id1 = bodies.add(d1);
        const BodyId id2 = bodies.add(d2);

        const SpinBody3D a = sb(d1.mu, d1.radius, d1.position, d1.velocity, {0.0, 0.0, d1.spinRate}, d1.inertiaFactor);
        const SpinBody3D b = sb(d2.mu, d2.radius, d2.position, d2.velocity, {0.0, 0.0, d2.spinRate}, d2.inertiaFactor);
        const auto ref = mergeBodies(bodies, id1, id2, 0.0);
        REQUIRE(ref.has_value());
        REQUIRE(ref->survivor == id1);
        const std::size_t s = *bodies.find(id1);
        const Merge3D m = mergeSpin3D(a, b, bodies.radius[s]);

        CHECK(m.mu == bodies.mu[s]);
        CHECK(relErr(m.r, {bodies.state.x[s], bodies.state.y[s], bodies.state.z[s]}) < 1e-12);
        CHECK(relErr(m.v, {bodies.state.vx[s], bodies.state.vy[s], bodies.state.vz[s]}) < 1e-12);
        CHECK(m.spin.x == 0.0);
        CHECK(m.spin.y == 0.0);
        CHECK(near(m.spin.z, bodies.spinRate[s], 1e-12));
        CHECK(near(m.omegaRaw.z, ref->omegaRaw, 1e-12));
        CHECK(near(m.omegaBreak, ref->omegaBreak, 1e-12));
        CHECK(m.spinClamped == ref->spinClamped);
        CHECK(m.lostAngularMomentumG.x == 0.0);
        CHECK(m.lostAngularMomentumG.y == 0.0);
        CHECK(std::abs(m.lostAngularMomentumG.z - ref->lostAngularMomentumG.z) <=
              1e-12 * std::max(1.0, norm(m.spinAngularMomentumG)));
    }
}

TEST_CASE("[3d-core] merge: off-plane, the vector spin keeps what the 2D merge logs as lost" * doctest::skip())
{
    const SpinBody3D a = bigOne(), b = smallOne();
    Bodies bodies;
    auto add = [&](const SpinBody3D& s, const char* name) {
        BodyDesc d;
        d.name = name;
        d.mu = s.mu;
        d.radius = s.radius;
        d.inertiaFactor = s.inertiaFactor;
        d.spinRate = s.spin.z; // 2D keeps only the z part
        d.position = s.r;
        d.velocity = s.v;
        return bodies.add(d);
    };
    const BodyId ia = add(a, "a");
    const BodyId ib = add(b, "b");
    const auto ref = mergeBodies(bodies, ia, ib, 0.0);
    REQUIRE(ref.has_value());
    REQUIRE_FALSE(ref->spinClamped);
    const std::size_t s = *bodies.find(ia);

    // 2D drops the x/y of the input spins too, so compare against a 3D merge of z-only spins.
    SpinBody3D az = a, bz = b;
    az.spin = {0.0, 0.0, a.spin.z};
    bz.spin = {0.0, 0.0, b.spin.z};
    const Merge3D m = mergeSpin3D(az, bz, bodies.radius[s]);
    const double I = m.inertiaFactor * m.mu * m.radius * m.radius;
    CHECK(near(I * m.spin.x, ref->lostAngularMomentumG.x, 1e-12));
    CHECK(near(I * m.spin.y, ref->lostAngularMomentumG.y, 1e-12));
    CHECK(near(m.spin.z, bodies.spinRate[s], 1e-12));
}
