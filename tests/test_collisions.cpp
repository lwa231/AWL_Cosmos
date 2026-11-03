#include <doctest.h>
#include <cmath>
#include "core/bodies.h"
#include "core/collisions.h"

using namespace cosmos;

namespace {
BodyDesc body(double mu, double radius, Vec3 p, Vec3 v)
{
    BodyDesc d;
    d.mu = mu;
    d.radius = radius;
    d.position = p;
    d.velocity = v;
    return d;
}
}

TEST_SUITE("collisions")
{
    TEST_CASE("head-on approach: contact time is (distance - R) / speed")
    {
        // 10 apart, closing at 2, contact at distance 3 -> t* = 3.5
        const auto t = sweptContactTime({10, 0, 0}, {-2, 0, 0}, 3.0, 100.0);
        REQUIRE(t.has_value());
        CHECK(*t == doctest::Approx(3.5).epsilon(1e-15).scale(0));
    }

    TEST_CASE("contact beyond the step is not reported, within it is")
    {
        CHECK(!sweptContactTime({10, 0, 0}, {-2, 0, 0}, 3.0, 3.4).has_value());
        CHECK(sweptContactTime({10, 0, 0}, {-2, 0, 0}, 3.0, 3.5).has_value());
        CHECK(sweptContactTime({10, 0, 0}, {-2, 0, 0}, 3.0, 3.6).has_value());
    }

    TEST_CASE("already overlapping or touching gives t* = 0, even when moving apart or at rest")
    {
        CHECK(*sweptContactTime({1, 0, 0}, {5, 0, 0}, 2.0, 1.0) == 0.0);
        CHECK(*sweptContactTime({1, 0, 0}, {0, 0, 0}, 2.0, 1.0) == 0.0);
        CHECK(*sweptContactTime({2, 0, 0}, {0, 3, 0}, 2.0, 1.0) == 0.0); // exactly touching
        CHECK(*sweptContactTime({0, 0, 0}, {0, 0, 0}, 0.0, 1.0) == 0.0);  // coincident point masses
    }

    TEST_CASE("separating or stationary pairs never collide")
    {
        CHECK(!sweptContactTime({10, 0, 0}, {2, 0, 0}, 3.0, 1e9).has_value());
        CHECK(!sweptContactTime({10, 0, 0}, {0, 0, 0}, 3.0, 1e9).has_value());
        CHECK(!sweptContactTime({10, 0, 0}, {0, 1, 0}, 3.0, 1e9).has_value());
    }

    TEST_CASE("a miss by closest approach is not a hit; a graze is")
    {
        // passing at impact parameter 4 along x
        CHECK(!sweptContactTime({100, 4, 0}, {-1, 0, 0}, 3.9, 1000.0).has_value());
        const auto t = sweptContactTime({100, 4, 0}, {-1, 0, 0}, 4.1, 1000.0);
        REQUIRE(t.has_value());
        const double expected = 100.0 - std::sqrt(4.1 * 4.1 - 16.0);
        CHECK(*t == doctest::Approx(expected).epsilon(1e-14).scale(0));
    }

    TEST_CASE("matches the plan's closed form (-B - sqrt(B^2 - AC)) / A")
    {
        const Vec3 p0{37.0, -12.5, 4.0}, w{-3.1, 1.2, -0.4};
        const double R = 9.0;
        const double A = dot(w, w), B = dot(p0, w), C = dot(p0, p0) - R * R;
        const double plan = (-B - std::sqrt(B * B - A * C)) / A;
        const auto t = sweptContactTime(p0, w, R, 1e6);
        REQUIRE(t.has_value());
        CHECK(*t == doctest::Approx(plan).epsilon(1e-13).scale(0));
        // and the position at t* is on the contact sphere
        const Vec3 p = p0 + w * (*t);
        CHECK(norm(p) == doctest::Approx(R).epsilon(1e-13).scale(0));
    }

    TEST_CASE("long step, tiny miss distance: no cancellation error")
    {
        // closing at 1e6 over 1e9: A C << B^2, where (-B - sqrt(disc)) / A would lose most of its digits
        const Vec3 p0{1e9, 0, 0}, w{-1e6, 1e-3, 0};
        const double R = 1.0;
        const auto t = sweptContactTime(p0, w, R, 1e4);
        REQUIRE(t.has_value());
        const Vec3 p = p0 + w * (*t);
        CHECK(norm(p) == doctest::Approx(R).epsilon(1e-9).scale(0));
    }

    TEST_CASE("no tunnelling: shuttle-like particle vs a thin body with a huge dt")
    {
        // 1e4 m/s particle, body radius 10 m, 1e6 m away. A single step of 1000 s would carry the particle
        // 1e7 m, ten times past the body, far outside it at both ends of the step.
        Bodies b;
        b.add(body(1e6, 10.0, {0, 0, 0}, {0, 0, 0}));        // thin body
        b.add(body(0.0, 0.0, {-1e6, 3.0, 0}, {1e4, 0, 0}));  // particle, impact parameter 3 m
        const double dt = 1000.0;
        // positions at the end of the step are well beyond the body: a start/end overlap test would see nothing
        const double endX = b.state.x[1] + b.state.vx[1] * dt;
        CHECK(endX > 8e6);

        const auto hits = findBodyContacts(b, dt);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].a == b.ids[0]);
        CHECK(hits[0].b == b.ids[1]);
        // contact when the particle reaches x = -sqrt(10^2 - 3^2): t = (1e6 - 9.539...) / 1e4
        CHECK(hits[0].t == doctest::Approx((1e6 - std::sqrt(91.0)) / 1e4).epsilon(1e-12).scale(0));

        // chord version (start + end states): the chord also passes through the body
        BodyState end = b.state;
        end.x[1] = endX;
        const auto chord = findBodyContacts(b, end, dt);
        REQUIRE(chord.size() == 1);
        CHECK(chord[0].t == doctest::Approx(hits[0].t).epsilon(1e-12).scale(0));

        // a particle that misses by 20 m is not reported
        b.state.y[1] = 20.0;
        end.y[1] = 20.0;
        CHECK(findBodyContacts(b, dt).empty());
        CHECK(findBodyContacts(b, end, dt).empty());
    }

    TEST_CASE("chord version for curved motion: relative path from start and end positions")
    {
        // Two bodies swing past each other; the chord from start to end positions crosses the contact radius
        Bodies b;
        b.add(body(1.0, 1.0, {0, 0, 0}, {0, 0, 0}));
        b.add(body(1.0, 1.0, {-10, 0.5, 0}, {0, 0, 0})); // start velocity says "at rest": start-velocity test misses
        BodyState end = b.state;
        end.x[1] = 10.0;
        CHECK(findBodyContacts(b, 1.0).empty());
        const auto c = findBodyContacts(b, end, 1.0);
        REQUIRE(c.size() == 1);
        // chord w = 20 per unit time, contact at |p| = 2 -> x = -sqrt(4 - 0.25)
        CHECK(c[0].t == doctest::Approx((10.0 - std::sqrt(3.75)) / 20.0).epsilon(1e-13).scale(0));
    }

    TEST_CASE("sweptContactTimeBetween with dt <= 0 reports only overlap")
    {
        CHECK(*sweptContactTimeBetween({1, 0, 0}, {50, 0, 0}, 2.0, 0.0) == 0.0);
        CHECK(!sweptContactTimeBetween({10, 0, 0}, {0, 0, 0}, 2.0, 0.0).has_value());
    }

    TEST_CASE("pairs come out in deterministic (t, id, id) order, independent of store order")
    {
        auto build = [](bool reversed) {
            Bodies b;
            struct P { double x; double vx; };
            // three equal-radius bodies on a line: 0 and 1 collide late, 1 and 2 earlier, 0 and 2 never within dt
            const P ps[] = {{0.0, 0.0}, {5.0, -1.0}, {9.0, -2.0}};
            if (!reversed) for (const P& p : ps) b.add(body(1.0, 0.5, {p.x, 0, 0}, {p.vx, 0, 0}));
            else for (int k = 2; k >= 0; --k) b.add(body(1.0, 0.5, {ps[k].x, 0, 0}, {ps[k].vx, 0, 0}));
            return b;
        };
        const Bodies fwd = build(false);
        const auto c = findBodyContacts(fwd, 10.0);
        REQUIRE(c.size() >= 2);
        for (std::size_t k = 1; k < c.size(); ++k) {
            const bool ordered = c[k - 1].t < c[k].t ||
                                 (c[k - 1].t == c[k].t && (c[k - 1].a < c[k].a || (c[k - 1].a == c[k].a && c[k - 1].b <= c[k].b)));
            CHECK(ordered);
        }
        // same physical pairs, in the same time order, from the reversed store (IDs differ, times agree)
        const Bodies rev = build(true);
        const auto r = findBodyContacts(rev, 10.0);
        REQUIRE(r.size() == c.size());
        for (std::size_t k = 0; k < c.size(); ++k) CHECK(r[k].t == c[k].t);
        // repeating the call gives the identical list
        const auto again = findBodyContacts(fwd, 10.0);
        REQUIRE(again.size() == c.size());
        for (std::size_t k = 0; k < c.size(); ++k) {
            CHECK(again[k].a == c[k].a);
            CHECK(again[k].b == c[k].b);
            CHECK(again[k].t == c[k].t);
        }
    }

    TEST_CASE("equal contact times are ordered by ID")
    {
        // a symmetric cross: two pairs meet at exactly the same time
        Bodies b;
        b.add(body(1.0, 1.0, {-10, 0, 0}, {1, 0, 0}));
        b.add(body(1.0, 1.0, {10, 0, 0}, {-1, 0, 0}));
        b.add(body(1.0, 1.0, {-10, 100, 0}, {1, 0, 0}));
        b.add(body(1.0, 1.0, {10, 100, 0}, {-1, 0, 0}));
        const auto c = findBodyContacts(b, 100.0);
        REQUIRE(c.size() == 2);
        CHECK(c[0].t == c[1].t);
        CHECK(c[0].a == b.ids[0]);
        CHECK(c[1].a == b.ids[2]);
    }

    TEST_CASE("already-overlapping pair is reported at t = 0")
    {
        Bodies b;
        b.add(body(1.0, 2.0, {0, 0, 0}, {0, 0, 0}));
        b.add(body(1.0, 2.0, {1, 0, 0}, {0, 0, 0}));
        const auto c = findBodyContacts(b, 0.5);
        REQUIRE(c.size() == 1);
        CHECK(c[0].t == 0.0);
    }
}
