#include <doctest.h>
#include <cmath>
#include <vector>
#include "core/events.h"
#include "core/impact.h"
#include "test_impact_util.h"

using namespace cosmos;
using namespace impact_test;

// Swept shuttle-vs-body test over one accepted step, atmosphere entry and the event helpers (plan 5.11, 8 items 1 and 3).

namespace {
const Vec3 O{0, 0, 0};

Atmosphere earthAir()
{
    Atmosphere a;
    a.rho0 = 1.225;
    a.scaleHeight = 7000.0;
    return a;
}
} // namespace

TEST_SUITE("impact_swept")
{
    TEST_CASE("a fast shuttle against a thin body in one huge step is not missed (no tunnelling)")
    {
        // 10 km/s for 1000 s = 10 000 km in one step, against a body of radius 1 km: both end points are far outside
        const double v = 1e4, dt = 1000.0, R = 1000.0, y = 300.0;
        const Vec3 s0{-5.0e6, y, 0.0}, s1{-5.0e6 + v * dt, y, 0.0};
        REQUIRE(norm(s0) > 1e6);
        REQUIRE(norm(s1) > 1e6);
        const SweptShuttleBody c = sweptShuttleVsBody(s0, s1, O, O, dt, R, 0.0, 0.0);
        REQUIRE(c.surface.has_value());
        const double exact = (5.0e6 - std::sqrt(R * R - y * y)) / v;
        CHECK(*c.surface == doctest::Approx(exact).epsilon(1e-11)); // measured 2.3e-13: the path is 1e4 body radii long
        CHECK(!c.atmosphere.has_value()); // no atmosphere
        CHECK(!c.startedInAtmosphere);
        // the contact point is on the sphere
        CHECK(norm(c.surfacePoint) == doctest::Approx(R).epsilon(1e-7)); // measured 1.1e-9 (ulp of 5e6 m)
        CHECK(c.surfacePoint.x < 0.0); // entered from the left

        // The same shuttle passing 1.5 R away: a miss
        const Vec3 m0{-5.0e6, 1500.0, 0.0}, m1{5.0e6, 1500.0, 0.0};
        const SweptShuttleBody miss = sweptShuttleVsBody(m0, m1, O, O, dt, R, 0.0, 0.0);
        CHECK(!miss.surface.has_value());
        // Grazing: the closest approach exactly R touches (the quadratic's discriminant is 0)
        const SweptShuttleBody graze = sweptShuttleVsBody({-5.0e6, R, 0}, {5.0e6, R, 0}, O, O, dt, R, 0.0, 0.0);
        CHECK(graze.surface.has_value());
        // Just short of the step end: the contact would come after the step
        const SweptShuttleBody notYet = sweptShuttleVsBody(s0, {-5.0e6 + 0.4e6, y, 0}, O, O, dt, R, 0.0, 0.0);
        CHECK(!notYet.surface.has_value());
    }

    TEST_CASE("the shuttle radius is added to the body radius")
    {
        const double dt = 10.0;
        const Vec3 s0{100, 0, 0}, s1{0, 0, 0}; // heads straight in at 10 per second
        const SweptShuttleBody a = sweptShuttleVsBody(s0, s1, O, O, dt, 40.0, 0.0, 0.0);
        const SweptShuttleBody b = sweptShuttleVsBody(s0, s1, O, O, dt, 40.0, 5.0, 0.0);
        REQUIRE(a.surface.has_value());
        REQUIRE(b.surface.has_value());
        CHECK(*a.surface == doctest::Approx(6.0).epsilon(1e-14));
        CHECK(*b.surface == doctest::Approx(5.5).epsilon(1e-14));
    }

    TEST_CASE("a moving body: the test uses the relative chord (body start and end positions)")
    {
        // Shuttle at rest at the origin; a body of radius 50 flies past along y = 20 from x = -1e5 to +1e5 in 100 s
        const double dt = 100.0, R = 50.0;
        const Vec3 b0{-1.0e5, 20.0, 0.0}, b1{1.0e5, 20.0, 0.0};
        const SweptShuttleBody c = sweptShuttleVsBody(O, O, b0, b1, dt, R, 0.0, 0.0);
        REQUIRE(c.surface.has_value());
        const double vb = 2.0e3; // body speed
        const double exact = (1.0e5 - std::sqrt(R * R - 20.0 * 20.0)) / vb;
        CHECK(*c.surface == doctest::Approx(exact).epsilon(1e-11)); // measured 2.3e-13: the path is 1e4 body radii long
        // Same relative motion with the shuttle moving instead: identical time
        const SweptShuttleBody d = sweptShuttleVsBody(b0 * -1.0 + Vec3{0, 40.0, 0}, b1 * -1.0 + Vec3{0, 40.0, 0}, O, O, dt, R, 0.0, 0.0);
        // (mirrored geometry: shuttle goes from +1e5 to -1e5 along y = 20 past a body at rest)
        REQUIRE(d.surface.has_value());
        CHECK(*d.surface == doctest::Approx(exact).epsilon(1e-13));
        // Both moving the same way (no relative motion) never touch, however fast
        const SweptShuttleBody e = sweptShuttleVsBody(O, {2.0e5, 0, 0}, b0 + Vec3{0, 500, 0}, b1 + Vec3{0, 500, 0}, dt, R, 0.0, 0.0);
        CHECK(!e.surface.has_value());
    }

    TEST_CASE("already overlapping at the start gives contact time 0")
    {
        const SweptShuttleBody c = sweptShuttleVsBody({10, 0, 0}, {500, 0, 0}, O, O, 5.0, 40.0, 0.0, 0.0);
        REQUIRE(c.surface.has_value());
        CHECK(*c.surface == 0.0);
        CHECK(c.surfacePoint.x == 10.0);
    }

    TEST_CASE("atmosphere entry is detected before the surface contact")
    {
        const double R = 6.371e6;
        const std::optional<Atmosphere> atm = earthAir();
        const double top = triggerAtmosphereTop(atm);
        REQUIRE(top > 90e3);
        REQUIRE(top < 105e3);
        // Radial dive at 8 km/s, step of 40 s: from 400 km altitude to 80 km under the surface-ish
        const double v = 8000.0, dt = 100.0;
        const Vec3 s0{R + 400e3, 0, 0}, s1{R + 400e3 - v * dt, 0, 0}; // ends 400 km deeper: inside the planet
        const SweptShuttleBody c = sweptShuttleVsBody(s0, s1, O, O, dt, R, 2.0, top);
        REQUIRE(c.atmosphere.has_value());
        REQUIRE(c.surface.has_value());
        CHECK(!c.startedInAtmosphere);
        const double tAtm = (400e3 - top) / v;
        const double tSurf = (400e3 - 2.0) / v;
        CHECK(*c.atmosphere == doctest::Approx(tAtm).epsilon(1e-12));
        CHECK(*c.surface == doctest::Approx(tSurf).epsilon(1e-12));
        CHECK(*c.atmosphere < *c.surface);
        CHECK(*c.surface - *c.atmosphere == doctest::Approx((top - 2.0) / v).epsilon(1e-9));

        // A shorter step that ends inside the air but above the ground: entry only
        const SweptShuttleBody air = sweptShuttleVsBody(s0, {R + 200e3 - 150e3, 0, 0}, O, O, 30.0, R, 2.0, top);
        REQUIRE(air.atmosphere.has_value());
        CHECK(!air.surface.has_value());
        // A step that stays above the air: nothing
        const SweptShuttleBody high = sweptShuttleVsBody(s0, {R + 300e3, 0, 0}, O, O, 10.0, R, 2.0, top);
        CHECK(!high.atmosphere.has_value());
        CHECK(!high.surface.has_value());
        CHECK(!high.startedInAtmosphere);

        // Starting inside the air: no new entry (it was reported when it happened), but the surface is still tested
        const SweptShuttleBody inside = sweptShuttleVsBody({R + 50e3, 0, 0}, {R - 10e3, 0, 0}, O, O, 10.0, R, 2.0, top);
        CHECK(inside.startedInAtmosphere);
        CHECK(!inside.atmosphere.has_value());
        REQUIRE(inside.surface.has_value());
        CHECK(*inside.surface == doctest::Approx(10.0 * (50e3 - 2.0) / 60e3).epsilon(1e-12));

        // A grazing pass through the air only
        const double b = R + top - 5e3; // closest approach 5 km inside the top, 93 km above the ground
        const SweptShuttleBody graze = sweptShuttleVsBody({-1.0e6, b, 0}, {1.0e6, b, 0}, O, O, 200.0, R, 2.0, top);
        CHECK(graze.atmosphere.has_value());
        CHECK(!graze.surface.has_value());
    }

    TEST_CASE("atmosphere crossing with a body that moves: relative geometry, and a thin atmosphere is ignored")
    {
        const double R = 1.0e6, top = 50e3;
        // Body falls toward a hovering shuttle at 1 km/s: b(t) = (3e6 - 1000 t, 0); shuttle at the origin
        const double dt = 3000.0;
        const Vec3 b0{3.0e6, 0, 0}, b1{0.0, 0, 0};
        const SweptShuttleBody c = sweptShuttleVsBody(O, O, b0, b1, dt, R, 0.0, top);
        REQUIRE(c.atmosphere.has_value());
        REQUIRE(c.surface.has_value());
        CHECK(*c.atmosphere == doctest::Approx((3.0e6 - (R + top)) / 1000.0).epsilon(1e-13));
        CHECK(*c.surface == doctest::Approx((3.0e6 - R) / 1000.0).epsilon(1e-13));
        // zero or negative top: no atmosphere crossing
        CHECK(!sweptShuttleVsBody(O, O, b0, b1, dt, R, 0.0, 0.0).atmosphere.has_value());
        CHECK(!sweptShuttleVsBody(O, O, b0, b1, dt, R, 0.0, -5.0).atmosphere.has_value());
    }

    TEST_CASE("degenerate step lengths do not crash and only report overlap")
    {
        const SweptShuttleBody a = sweptShuttleVsBody({100, 0, 0}, {0, 0, 0}, O, O, 0.0, 10.0, 0.0, 0.0);
        CHECK(!a.surface.has_value());
        const SweptShuttleBody b = sweptShuttleVsBody({5, 0, 0}, {5, 0, 0}, O, O, 0.0, 10.0, 0.0, 0.0);
        REQUIRE(b.surface.has_value());
        CHECK(*b.surface == 0.0);
        CHECK(b.surfacePoint.x == 5.0);
    }

    TEST_CASE("events: AtmosphereEntry then ShuttleDestroyed, at step start + contact time")
    {
        SweptShuttleBody c;
        c.atmosphere = 12.5;
        c.surface = 14.0;
        std::vector<Event> ev;
        appendShuttleStepEvents(ev, 1000.0, 7, 3, c);
        REQUIRE(ev.size() == 2);
        CHECK(ev[0].type == EventType::AtmosphereEntry);
        CHECK(ev[0].t == 1012.5);
        CHECK(ev[0].id == 7);
        CHECK(ev[0].other == 3);
        CHECK(ev[1].type == EventType::ShuttleDestroyed);
        CHECK(ev[1].t == 1014.0);
        CHECK(ev[1].id == 7);
        CHECK(ev[1].other == 3);

        std::vector<Event> none;
        appendShuttleStepEvents(none, 5.0, 1, 2, SweptShuttleBody{});
        CHECK(none.empty());

        SweptShuttleBody only;
        only.atmosphere = 1.0;
        appendShuttleStepEvents(none, 5.0, 1, 2, only);
        REQUIRE(none.size() == 1);
        CHECK(none[0].type == EventType::AtmosphereEntry);

        // Equal to the event makers
        const Event d = makeShuttleDestroyed(1014.0, 7, 3);
        CHECK(!eventBefore(d, ev[1]));
        CHECK(!eventBefore(ev[1], d));
    }

    TEST_CASE("findShuttleContacts and destruction by the earliest body only")
    {
        // Planet P (id 1, R = 1000, air top ~ 100) at (0,0); moon M (id 2, R = 100) at (3000, 0); far body F (id 3).
        // A shuttle flies along y = 0 from x = -5000 to x = 5000 in dt = 100 s (100 per second): the planet comes first.
        Bodies b;
        BodyDesc p = point(1.0, O, O, 1000.0);
        Atmosphere at;
        at.rho0 = 1.0;
        at.scaleHeight = 100.0 / std::log(1.0 / kAtmosphereCutoffDensity); // top = 100 exactly
        p.atmosphere = at;
        b.add(p);
        b.add(point(0.1, {3000, 0, 0}, O, 100.0));
        b.add(point(0.1, {0, 90000, 0}, O, 100.0));
        REQUIRE(triggerAtmosphereTop(b.atmosphere[0]) == doctest::Approx(100.0).epsilon(1e-12));

        const Vec3 s0{-5000, 0, 0}, s1{5000, 0, 0};
        std::vector<ShuttleBodyHit> hits;
        findShuttleContacts(s0, s1, 0.0, b, b.state, 100.0, hits);
        REQUIRE(hits.size() == 2); // the far body is not touched
        // planet: air entry at x = -1100 (t = 39), surface at x = -1000 (t = 40); moon: surface at x = 2900 (t = 79)
        CHECK(hits[0].body == b.ids[0]);
        CHECK(hits[1].body == b.ids[1]);
        REQUIRE(hits[0].contact.atmosphere.has_value());
        REQUIRE(hits[0].contact.surface.has_value());
        CHECK(*hits[0].contact.atmosphere == doctest::Approx(39.0).epsilon(1e-12));
        CHECK(*hits[0].contact.surface == doctest::Approx(40.0).epsilon(1e-12));
        CHECK(!hits[1].contact.atmosphere.has_value());
        CHECK(*hits[1].contact.surface == doctest::Approx(79.0).epsilon(1e-12));

        // Events: destroyed by the planet at t = 40; the moon's later contact is ignored
        std::vector<Event> ev;
        CHECK(appendShuttleStepEvents(ev, 10.0, 99, hits));
        REQUIRE(ev.size() == 2);
        CHECK(ev[0].type == EventType::AtmosphereEntry);
        CHECK(ev[0].t == doctest::Approx(49.0).epsilon(1e-12));
        CHECK(ev[1].type == EventType::ShuttleDestroyed);
        CHECK(ev[1].t == doctest::Approx(50.0).epsilon(1e-12));
        CHECK(ev[1].other == b.ids[0]);

        // Reusing the output vector keeps its capacity and clears the old content
        hits.reserve(16);
        const ShuttleBodyHit* before = hits.data();
        findShuttleContacts({0, 5000, 0}, {0, 6000, 0}, 0.0, b, b.state, 10.0, hits);
        CHECK(hits.empty());
        CHECK(hits.data() == before);

        // A shuttle that only brushes the moon's side: no planet events
        findShuttleContacts({2950, 5000, 0}, {2950, -5000, 0}, 0.0, b, b.state, 100.0, hits);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].body == b.ids[1]);
        ev.clear();
        CHECK(appendShuttleStepEvents(ev, 0.0, 99, hits));
        REQUIRE(ev.size() == 1);
        CHECK(ev[0].type == EventType::ShuttleDestroyed);

        // Entry without destruction
        findShuttleContacts({-5000, 600, 0}, {5000, 600, 0}, 0.0, b, b.state, 100.0, hits);
        // y = 600 < 1000: that is a hit again; use a path that skims the air at y = 1050 (inside the top at 1100)
        findShuttleContacts({-5000, 1050, 0}, {5000, 1050, 0}, 0.0, b, b.state, 100.0, hits);
        REQUIRE(hits.size() == 1);
        ev.clear();
        CHECK(!appendShuttleStepEvents(ev, 0.0, 99, hits));
        REQUIRE(ev.size() == 1);
        CHECK(ev[0].type == EventType::AtmosphereEntry);
    }

    TEST_CASE("surface contact earliest wins; atmosphere entries after the destruction are dropped; ties by id")
    {
        ShuttleBodyHit a, b2, c;
        a.body = 5;
        a.contact.surface = 30.0;
        a.contact.atmosphere = 20.0;
        b2.body = 4;
        b2.contact.surface = 30.0; // same time, smaller id: this one destroys
        b2.contact.atmosphere = 25.0;
        c.body = 9;
        c.contact.atmosphere = 31.0; // after the destruction: dropped
        const std::vector<ShuttleBodyHit> hits{a, b2, c};
        std::vector<Event> ev;
        CHECK(appendShuttleStepEvents(ev, 0.0, 1, hits));
        REQUIRE(ev.size() == 3);
        EventQueue q;
        q.append(ev);
        q.sort();
        CHECK(q.events()[0].type == EventType::AtmosphereEntry);
        CHECK(q.events()[0].other == 5);
        CHECK(q.events()[1].type == EventType::AtmosphereEntry);
        CHECK(q.events()[1].other == 4);
        CHECK(q.events()[2].type == EventType::ShuttleDestroyed);
        CHECK(q.events()[2].other == 4);

        // Input order does not matter
        const std::vector<ShuttleBodyHit> rev{c, b2, a};
        std::vector<Event> ev2;
        CHECK(appendShuttleStepEvents(ev2, 0.0, 1, rev));
        EventQueue q2;
        q2.append(ev2);
        q2.sort();
        REQUIRE(q2.size() == q.size());
        for (std::size_t i = 0; i < q.size(); ++i) {
            CHECK(q2.events()[i].type == q.events()[i].type);
            CHECK(q2.events()[i].other == q.events()[i].other);
            CHECK(q2.events()[i].t == q.events()[i].t);
        }
    }

    TEST_CASE("with dense output: a planet on an orbit, body positions at both ends of a step from IAS15")
    {
        // Star (mu = 1) at the origin, planet (massless here) on a circular orbit r = 10, radius 0.15; a shuttle moves
        // in a straight line through the planet's path. The step is 1/100 of an orbit; the chord test is compared with
        // a root find on the true (dense) relative distance.
        Bodies b;
        b.add(point(1.0, O, O));
        const double r0 = 10.0, vc = std::sqrt(1.0 / r0), Rp = 0.15;
        b.add(point(0.0, {r0, 0, 0}, {0, vc, 0}, Rp));
        IAS15 ias;
        BodyState s = b.state;
        const ForceFn force = [&b](const BodyState& st, Accel& a) { computeAccelerations(st, b.mu, a); };
        double t = 0.0;
        const double period = 2.0 * kPi * std::pow(r0, 1.5);
        const double dtStep = period / 100.0;
        ias.step(s, force, t, dtStep);
        while (t < dtStep) ias.step(s, force, t, dtStep);
        REQUIRE(ias.hasDenseOutput());
        // The dense output of the last step covers [stepStart, stepEnd]; use the whole span of the last step
        const double tA = ias.stepStartTime(), tB = ias.stepEndTime(), dt = tB - tA;
        BodyState startState, endState;
        ias.stateAt(0.0, startState);
        ias.stateAt(1.0, endState);
        Bodies at = b;
        at.state = startState;

        // Shuttle moves at speed 1.0 along (0.6, 0.8), five times the planet's speed times 30. Aim it so that its line
        // passes the planet's position at the middle of the step.
        const Vec3 pm = ias.positionOf(1, 0.5);
        const Vec3 dir{0.6, 0.8, 0.0};
        const double sp = 1.0;
        const Vec3 sMid = pm; // shuttle at the planet's centre at the middle of the step ...
        const Vec3 sA = sMid - dir * (sp * 0.5 * dt) + Vec3{0.0, 0.0, 0.0};
        // ... displaced sideways by 0.05 so that the pass is a graze of radius < Rp
        const Vec3 off = Vec3{-dir.y, dir.x, 0.0} * 0.05;
        const Vec3 s0 = sA + off, s1 = sA + dir * (sp * dt) + off;

        std::vector<ShuttleBodyHit> hits;
        findShuttleContacts(s0, s1, 0.0, at, endState, dt, hits);
        REQUIRE(hits.size() == 1);
        REQUIRE(hits[0].contact.surface.has_value());
        const double chord = *hits[0].contact.surface;

        // Reference: bisection on the true distance |s(tau) - b(tau)| (shuttle straight and uniform, body dense output)
        auto dist = [&](double tau) {
            const Vec3 sp_ = s0 + (s1 - s0) * tau;
            return norm(sp_ - ias.positionOf(1, tau));
        };
        double lo = 0.0, hi = 0.5;
        REQUIRE(dist(0.0) > Rp);
        REQUIRE(dist(hi) < Rp);
        for (int k = 0; k < 100; ++k) {
            const double mid = 0.5 * (lo + hi);
            (dist(mid) > Rp ? lo : hi) = mid;
        }
        const double truth = 0.5 * (lo + hi) * dt;
        // The planet's arc over one step is curved (sagitta ~ r phi^2 / 8 = 1e-3 r_orbit/... ), the chord is not: the
        // contact times agree to a small fraction of the step (measured 1.1e-3 of the step; require 1e-2)
        std::printf("[impact] chord vs dense contact time: %.3e of the step\n", std::abs(chord - truth) / dt);
        CHECK(std::abs(chord - truth) < 1e-2 * dt);
        CHECK(chord > 0.0);
        CHECK(chord < dt);
    }
}
