// [3d-core] spec: swept shuttle-vs-body and body-body tests in 3D. These functions (collisions.h, impact.h) already
// work on full 3D vectors, so there is nothing new to declare: the spec pins that behaviour out of plane. Skipped by
// default; run with
//   cosmos_tests --no-skip --test-case=*3d-core*swept*
#include <doctest.h>
#include <cmath>
#include <vector>
#include "core/bodies.h"
#include "core/collisions.h"
#include "core/impact.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {
const double kR = 6.371e6;
const double kDt = 100.0;
} // namespace

TEST_CASE("[3d-core] swept: a pass over the body that crosses it in projection but misses in 3D" * doctest::skip())
{
    // The chord runs over the body's centre in x-y, but 1.4 R above it in z.
    const Vec3 b{0.0, 0.0, 0.0};
    const Vec3 s0{-2.0e7, 0.0, 1.4 * kR}, s1{2.0e7, 0.0, 1.4 * kR};
    const SweptShuttleBody c = sweptShuttleVsBody(s0, s1, b, b, kDt, kR, 10.0, 0.0);
    CHECK_FALSE(c.surface.has_value());
    CHECK_FALSE(c.atmosphere.has_value());

    // With a thick enough atmosphere (top 0.5 R) it passes through the air, but still not the surface.
    const SweptShuttleBody a = sweptShuttleVsBody(s0, s1, b, b, kDt, kR, 10.0, 0.5 * kR);
    CHECK_FALSE(a.surface.has_value());
    REQUIRE(a.atmosphere.has_value());
    const double x = -std::sqrt(1.5 * kR * 1.5 * kR - 1.4 * kR * 1.4 * kR);
    CHECK(*a.atmosphere == doctest::Approx((x + 2.0e7) / 4.0e7 * kDt).epsilon(1e-12));
}

TEST_CASE("[3d-core] swept: a low pass hits, at the 3D contact point" * doctest::skip())
{
    const double z = 0.6 * kR, rs = 10.0;
    const Vec3 b{0.0, 0.0, 0.0};
    const Vec3 s0{-2.0e7, 0.0, z}, s1{2.0e7, 0.0, z};
    const SweptShuttleBody c = sweptShuttleVsBody(s0, s1, b, b, kDt, kR, rs, 0.0);
    REQUIRE(c.surface.has_value());
    const double R = kR + rs;
    const double x = -std::sqrt(R * R - z * z);
    CHECK(*c.surface == doctest::Approx((x + 2.0e7) / 4.0e7 * kDt).epsilon(1e-12));
    CHECK(relErr(c.surfacePoint, {x, 0.0, z}) < 1e-12);
    CHECK(norm(c.surfacePoint - b) == doctest::Approx(R).epsilon(1e-12));
}

TEST_CASE("[3d-core] swept: a moving body and a generic orientation give the same times as the flat case" *
          doctest::skip())
{
    // Shuttle and body both move; the relative chord is what counts. Then the whole thing turned by rotQ.
    const Vec3 s0{-2.0e7, 3.0e6, 0.0}, s1{2.0e7, 3.0e6, 0.0};
    const Vec3 b0{1.0e6, 0.0, 0.0}, b1{-1.0e6, 0.0, 0.0};
    const SweptShuttleBody flat = sweptShuttleVsBody(s0, s1, b0, b1, kDt, kR, 10.0, 1.0e6);
    const SweptShuttleBody tilt = sweptShuttleVsBody(rotQ(s0), rotQ(s1), rotQ(b0), rotQ(b1), kDt, kR, 10.0, 1.0e6);
    REQUIRE(flat.surface.has_value());
    REQUIRE(tilt.surface.has_value());
    REQUIRE(flat.atmosphere.has_value());
    REQUIRE(tilt.atmosphere.has_value());
    CHECK(*tilt.surface == doctest::Approx(*flat.surface).epsilon(1e-12));
    CHECK(*tilt.atmosphere == doctest::Approx(*flat.atmosphere).epsilon(1e-12));
    CHECK(*tilt.atmosphere <= *tilt.surface);
    CHECK(relErr(tilt.surfacePoint, rotQ(flat.surfacePoint)) < 1e-12);
}

TEST_CASE("[3d-core] swept: findShuttleContacts and findBodyContacts see bodies stacked in z" * doctest::skip())
{
    // Two bodies with the same x and y, at z = +-3 R. A shuttle falling along z between them hits only the lower one
    // if it moves down, and the chord passes the x-y position of both.
    Bodies bodies;
    BodyDesc up;
    up.name = "up";
    up.mu = 1.0e14;
    up.radius = kR;
    up.position = {0.0, 0.0, 3.0 * kR};
    BodyDesc down = up;
    down.name = "down";
    down.position = {0.0, 0.0, -3.0 * kR};
    const BodyId idUp = bodies.add(up);
    const BodyId idDown = bodies.add(down);
    (void)idUp;

    const Vec3 s0{0.0, 0.0, 0.0}, s1{0.0, 0.0, -2.5 * kR};
    std::vector<ShuttleBodyHit> hits;
    findShuttleContacts(s0, s1, 10.0, bodies, bodies.state, kDt, hits);
    REQUIRE(hits.size() == 1);
    CHECK(hits[0].body == idDown);
    REQUIRE(hits[0].contact.surface.has_value());
    CHECK(*hits[0].contact.surface == doctest::Approx((2.0 * kR - 10.0) / (2.5 * kR) * kDt).epsilon(1e-12));

    // Body-body: the two close along z at 2e5 m/s each and touch when the gap 4 R is closed.
    bodies.state.vz[0] = -2.0e5;
    bodies.state.vz[1] = 2.0e5;
    const auto contacts = findBodyContacts(bodies, kDt);
    REQUIRE(contacts.size() == 1);
    CHECK(contacts[0].t == doctest::Approx(4.0 * kR / 4.0e5).epsilon(1e-12));

    // And sweptContactTime directly, for a relative path entirely along z.
    const auto t = sweptContactTime({0.0, 0.0, 10.0 * kR}, {0.0, 0.0, -1.0e6}, kR, kDt);
    REQUIRE(t.has_value());
    CHECK(*t == doctest::Approx(9.0 * kR / 1.0e6).epsilon(1e-12));
}

TEST_CASE("[3d-core] swept: the closing-time estimate uses the 3D line of centres" * doctest::skip())
{
    // Closing along z only: x-y components of dp are 0, so a 2D estimate would see no separation at all.
    const Vec3 dp{0.0, 0.0, 1.0e7};
    const Vec3 dv{0.0, 0.0, -1.0e3};
    const auto t = estimateClosing(dp, dv, {0.0, 0.0, 0.0}, 2.0e6);
    REQUIRE(t.has_value());
    CHECK(*t == doctest::Approx(8.0e3).epsilon(1e-12));
}
