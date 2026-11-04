#include <doctest.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <set>
#include "test_app_overlays_util.h"

using namespace ovl_test;

namespace {

// A point circling (cx, cy) at radius R and angular rate w, the centre moving with (ux, uy): inertial sample at time t.
TrailSample circler(double t, double R, double w, double ux, double uy)
{
    TrailSample s;
    s.t = t;
    s.x = ux * t + R * std::cos(w * t);
    s.y = uy * t + R * std::sin(w * t);
    s.vx = ux - R * w * std::sin(w * t);
    s.vy = uy + R * w * std::cos(w * t);
    return s;
}

TrailSample mover(double t, double ux, double uy)
{
    return {t, ux * t, uy * t, ux, uy};
}

const LineStrip* stripEndingAt(const LineBatch& b, double x, double y, double tol)
{
    for (const LineStrip& s : b.strips) {
        const LineVertex& v = b.vertices[s.first + s.count - 1];
        if (std::hypot(v.x - x, v.y - y) < tol) return &s;
    }
    return nullptr;
}

} // namespace

TEST_SUITE("app overlays trails")
{
    TEST_CASE("steady straight motion is thinned to a couple of samples, a circle to about 100 per turn")
    {
        Trail line;
        for (int i = 0; i <= 2000; ++i) line.record(mover(i * 10.0, 3.0, 4.0), 1e9);
        CHECK(line.storedCount() <= 2);
        CHECK(line.head().t == 20000.0);                      // the head is the latest observation
        CHECK(line.lastTime() == 20000.0);

        Trail circle;
        const double w = 2.0 * kPi / 3600.0;                  // one turn per hour, sampled every 10 s
        for (int i = 0; i <= 360; ++i) circle.record(circler(i * 10.0, 1000.0, w, 0.0, 0.0), 1e9);   // exactly one turn
        CHECK(circle.storedCount() >= 80);                    // a sample whenever the velocity has turned 3 degrees: 4 of 10 s steps here
        CHECK(circle.storedCount() <= 125);
        // Timestamps are exactly the observation times and increase.
        for (std::size_t k = 1; k < circle.sampleCount(); ++k) CHECK(circle.sample(k).t > circle.sample(k - 1).t);
        CHECK(std::fmod(circle.sample(5).t, 10.0) == 0.0);
    }

    TEST_CASE("an arc-length spacing stores a sample when the object has moved far enough")
    {
        Trail t;
        for (int i = 0; i <= 100; ++i) t.record(mover(i, 1.0, 0.0), 10.0);   // 1 m per sample, 10 m spacing
        CHECK(t.storedCount() == 11);
        CHECK(t.stored(1).x == 10.0);
    }

    TEST_CASE("the ring keeps the newest kTrailCapacity samples, oldest first")
    {
        Trail t;
        const std::size_t total = kTrailCapacity + 500;
        for (std::size_t i = 0; i < total; ++i) t.record(mover(static_cast<double>(i), 1.0, 0.0), 0.0);
        REQUIRE(t.storedCount() == kTrailCapacity);
        CHECK(t.stored(0).t == 500.0);
        CHECK(t.stored(kTrailCapacity - 1).t == static_cast<double>(total - 1));
        for (std::size_t k = 1; k < kTrailCapacity; ++k) REQUIRE(t.stored(k).t == t.stored(k - 1).t + 1.0);
        t.clear();
        CHECK(t.empty());
        t.record(mover(7.0, 1.0, 0.0), 0.0);
        CHECK(t.storedCount() == 1);
        CHECK(t.stored(0).t == 7.0);
    }

    TEST_CASE("a trail of a circler around a moving centre is a closed circle in the centre's frame")
    {
        // Inertial samples of the circler and of its centre, each thinned on its own.
        const double R = 5000.0, w = 2.0 * kPi / 600.0, ux = 40.0, uy = -25.0;
        Trail obj, org;
        for (int i = 0; i <= 3000; ++i) {                    // 5 turns, sampled every 1 s
            const double t = i;
            obj.record(circler(t, R, w, ux, uy), 1e12);
            org.record(mover(t, ux, uy), 2000.0);              // the origin is sampled on its own (spacing) schedule
        }
        CHECK(org.storedCount() < obj.storedCount());

        TrailDrawParams p;
        p.now = 3000.0;
        p.metresPerPixel = 1.0;
        p.minStepPx = 0.0;
        LineBatch b;
        const std::size_t n = appendTrail(obj, &org, p, b);
        REQUIRE(n > 100);
        double worst = 0.0, sweep = 0.0, prev = std::atan2(b.vertices[0].y, b.vertices[0].x);
        for (std::size_t k = 0; k < n; ++k) {
            const LineVertex& v = b.vertices[k];
            worst = std::max(worst, std::abs(std::hypot(v.x, v.y) - R) / R);
            const double a = std::atan2(v.y, v.x);
            double da = a - prev;
            if (da > kPi) da -= 2.0 * kPi;
            if (da < -kPi) da += 2.0 * kPi;
            sweep += da;
            prev = a;
        }
        CHECK(worst < 1e-4);                                  // on the circle everywhere, including between samples
        CHECK(sweep / (2.0 * kPi) == doctest::Approx(5.0).epsilon(0.01));
        // Without the origin the same trail is a long wave far from the circle.
        LineBatch c;
        appendTrail(obj, nullptr, p, c);
        double far = 0.0;
        for (const LineVertex& v : c.vertices) far = std::max<double>(far, std::hypot(v.x, v.y));
        CHECK(far > 20.0 * R);
    }

    TEST_CASE("a nearly stationary object seen from a moving frame still draws the frame's curve")
    {
        // The object sits still (two samples); the origin circles it. Seen from the origin it must be a circle, which
        // needs the vertices at the origin's sample times.
        const double R = 1000.0, w = 2.0 * kPi / 100.0;
        Trail obj, org;
        for (int i = 0; i <= 200; ++i) {
            obj.record({static_cast<double>(i), 0.0, 0.0, 0.0, 0.0}, 1e12);
            org.record(circler(i, R, w, 0.0, 0.0), 1e12);
        }
        CHECK(obj.sampleCount() == 2);
        TrailDrawParams p;
        p.now = 200.0;
        p.minStepPx = 0.0;
        LineBatch b;
        const std::size_t n = appendTrail(obj, &org, p, b);
        REQUIRE(n > 60);
        for (std::size_t k = 0; k < n; ++k) CHECK(std::hypot(b.vertices[k].x, b.vertices[k].y) == doctest::Approx(R).epsilon(1e-4));
    }

    TEST_CASE("the trail fades towards the tail and can be limited in length")
    {
        Trail obj;
        for (int i = 0; i <= 1000; ++i) obj.record(circler(i, 100.0, 0.05, 0.0, 0.0), 0.0);
        TrailDrawParams p;
        p.now = 1000.0;
        p.minStepPx = 0.0;
        p.colour = {0.1f, 0.2f, 0.3f, 0.8f};
        LineBatch full;
        const std::size_t nFull = appendTrail(obj, nullptr, p, full);
        REQUIRE(nFull >= 100);
        CHECK(full.vertices.front().a == doctest::Approx(0.0f).epsilon(1e-6));
        CHECK(full.vertices.back().a == doctest::Approx(0.8f));
        CHECK(full.vertices.back().r == 0.1f);
        for (std::size_t k = 1; k < nFull; ++k) CHECK(full.vertices[k].a >= full.vertices[k - 1].a);   // monotone fade

        p.tailTime = 1000.0 - 200.0;                          // only the last 200 s
        LineBatch part;
        const std::size_t nPart = appendTrail(obj, nullptr, p, part);
        CHECK(nPart < nFull / 3);
        CHECK(part.vertices.front().a == doctest::Approx(0.0f).epsilon(1e-6));   // the fade is relative to the shown length
    }

    TEST_CASE("vertices closer than a pixel are dropped, the end vertex is always kept")
    {
        Trail obj;
        for (int i = 0; i <= 1000; ++i) obj.record(mover(i, 0.3, 0.0), 0.0);   // 0.3 m per sample
        TrailDrawParams p;
        p.now = 1000.0;
        p.metresPerPixel = 1.0;
        p.minStepPx = 1.0;
        LineBatch b;
        const std::size_t n = appendTrail(obj, nullptr, p, b);
        CHECK(n < 400);
        CHECK(b.vertices.back().x == doctest::Approx(300.0f));
        for (std::size_t k = 2; k + 1 < n; ++k) CHECK(b.vertices[k].x - b.vertices[k - 1].x >= 0.99f);
    }

    TEST_CASE("a moon circling a moving planet is a closed loop in the follow-body frame")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        OverlayBuilder ob;
        ob.settings.trails = true;
        const double mpp = 1e6;
        RenderFrame f = frameOf(2000, 1200);                 // the whole loop (radius 3.7e8 m) is inside the view: nothing is culled
        double rMin = 1e300, rMax = 0.0;
        const double dt = w.moonPeriod / 150.0;
        const int frames = 330;                              // 2.2 moon periods
        Vec3 planetNow, moonNow;
        for (int i = 1; i <= frames; ++i) {
            d.advanceTo(i * dt);
            const DisplayBody& pl = *bodyOf(d.state(), w.planet);
            const DisplayBody& mo = *bodyOf(d.state(), w.moon);
            planetNow = pl.r;
            moonNow = mo.r;
            const double r = std::hypot(mo.r.x - pl.r.x, mo.r.y - pl.r.y);
            rMin = std::min(rMin, r);
            rMax = std::max(rMax, r);
            f.clearObjects();
            OverlayView v = viewAt(pl.r, mpp);
            v.frameMode = FrameMode::FollowBody;
            v.frameBody = w.planet;
            ob.build(d.state(), d.sim(), 1, v, f);
        }
        // The moon's strip ends at the moon (camera-relative), whatever order the strips came in.
        const LineStrip* moon = stripEndingAt(f.underlay, moonNow.x - planetNow.x, moonNow.y - planetNow.y, 1e5);
        REQUIRE(moon != nullptr);
        CHECK(stripEndingAt(f.underlay, 0.0, 0.0, 1e5) == nullptr);       // the frame body has no trail of its own
        REQUIRE(moon->count > 200);
        double sweep = 0.0, prev = 0.0, worstLow = 1e300, worstHigh = 0.0;
        for (std::uint32_t k = 0; k < moon->count; ++k) {
            const LineVertex& v = f.underlay.vertices[moon->first + k];
            const double r = std::hypot(v.x, v.y), a = std::atan2(v.y, v.x);
            worstLow = std::min(worstLow, r);
            worstHigh = std::max(worstHigh, r);
            if (k > 0) {
                double da = a - prev;
                if (da > kPi) da -= 2.0 * kPi;
                if (da < -kPi) da += 2.0 * kPi;
                sweep += da;
            }
            prev = a;
        }
        CHECK(sweep / (2.0 * kPi) > 2.1);                                  // more than two full turns around the planet
        CHECK(worstLow > rMin * 0.999);                                    // every vertex lies on the true relative orbit
        CHECK(worstHigh < rMax * 1.001);
        CHECK(rMax / rMin < 1.2);

        // The same trail in the inertial frame is nowhere near a loop about the planet.
        f = frameOf(20000, 12000);                           // a view wide enough that the far part of the trail is not culled
        ob.build(d.state(), d.sim(), 1, viewAt(planetNow, mpp), f);
        const LineStrip* inertial = stripEndingAt(f.underlay, moonNow.x - planetNow.x, moonNow.y - planetNow.y, 1e5);
        REQUIRE(inertial != nullptr);
        double far = 0.0;
        for (std::uint32_t k = 0; k < inertial->count; ++k) {
            const LineVertex& v = f.underlay.vertices[inertial->first + k];
            far = std::max<double>(far, std::hypot(v.x, v.y));
        }
        CHECK(far > 10.0 * rMax);
    }

    TEST_CASE("removal, reset, a new generation and time running backwards clear trails")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        OverlayBuilder ob;
        ob.settings.trails = true;
        RenderFrame f = frameOf();
        for (int i = 1; i <= 20; ++i) {
            d.advanceTo(i * 1000.0);
            f.clearObjects();
            ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e6), f);
        }
        CHECK(ob.trailStore().size() == 4);                  // three bodies and the barycentre
        CHECK(ob.trailStore().find(w.moon) != nullptr);
        CHECK(ob.trailStore().find(w.moon)->sampleCount() > 5);

        ob.onObjectRemoved(w.moon);
        CHECK(ob.trailStore().find(w.moon) == nullptr);
        ob.onEvent(makeMerge(1.0, w.star, w.planet));         // the absorbed body is the second id
        CHECK(ob.trailStore().find(w.planet) == nullptr);
        CHECK(ob.trailStore().find(w.star) != nullptr);
        ob.onEvent(makeShuttleDestroyed(1.0, w.star, 0));
        CHECK(ob.trailStore().find(w.star) == nullptr);
        ob.onEvent(makeFuelEmpty(1.0, 99));                   // other events change nothing
        // Objects that are still in the state come back (with a fresh history) and absent ones stay gone.
        d.advanceTo(21000.0);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e6), f);
        CHECK(ob.trailStore().find(w.moon)->sampleCount() == 1);

        ob.reset();
        CHECK(ob.trailStore().size() == 0);

        // A new generation (Stop / rebuild) starts over.
        for (int i = 22; i <= 25; ++i) {
            d.advanceTo(i * 1000.0);
            f.clearObjects();
            ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e6), f);
        }
        CHECK(ob.trailStore().find(w.moon)->sampleCount() >= 4);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 2, viewAt({0, 0, 0}, 1e6), f);
        CHECK(ob.trailStore().find(w.moon)->sampleCount() == 1);

        // The display time going backwards (a restored snapshot) does too.
        d.advanceTo(30000.0);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 2, viewAt({0, 0, 0}, 1e6), f);
        const std::size_t before = ob.trailStore().find(w.moon)->sampleCount();
        CHECK(before >= 2);
        DisplayState earlier = d.state();
        earlier.t -= 5000.0;
        f.clearObjects();
        ob.build(earlier, d.sim(), 2, viewAt({0, 0, 0}, 1e6), f);
        CHECK(ob.trailStore().find(w.moon)->sampleCount() == 1);
    }

    TEST_CASE("trails off: nothing is drawn and the history is dropped; keepTrailsWhileHidden records anyway")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        OverlayBuilder ob;
        RenderFrame f = frameOf();
        auto frames = [&](int from, int to) {
            for (int i = from; i <= to; ++i) {
                d.advanceTo(i * 1000.0);
                f.clearObjects();
                ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e6), f);
            }
        };
        frames(1, 5);
        CHECK(f.underlay.empty());
        CHECK(ob.trailStore().size() == 0);                  // never recorded
        ob.settings.keepTrailsWhileHidden = true;
        frames(6, 10);
        CHECK(f.underlay.empty());                           // recorded but hidden
        CHECK(ob.trailStore().find(w.planet)->sampleCount() >= 5);
        ob.settings.trails = true;
        frames(11, 12);
        CHECK_FALSE(f.underlay.strips.empty());
        ob.settings.trails = false;
        ob.settings.keepTrailsWhileHidden = false;
        frames(13, 13);
        CHECK(ob.trailStore().size() == 0);
    }

    TEST_CASE("steady-state frames reuse the capacity of the batches")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        OverlayBuilder ob;
        ob.settings.trails = true;
        ob.settings.velocityArrows = true;
        ob.settings.accelerationArrows = true;
        ob.settings.osculatingOrbits = true;
        RenderFrame f = frameOf();
        auto frame = [&]() {
            f.clearObjects();
            OverlayView v = viewAt(bodyOf(d.state(), w.planet)->r, 1e6);
            v.frameMode = FrameMode::FollowBody;
            v.frameBody = w.planet;
            ob.build(d.state(), d.sim(), 1, v, f);
        };
        for (int i = 1; i <= 100; ++i) {
            d.advanceTo(i * 5000.0);
            frame();
        }
        frame();   // the same state again: the content size is now fixed
        const std::size_t capV = f.underlay.vertices.capacity(), capS = f.underlay.strips.capacity(),
                          capG = f.underlay.segments.capacity(), capO = f.overlay.segments.capacity();
        CHECK(capV > 0);
        CHECK(capO > 0);
        for (int i = 0; i < 3; ++i) {
            frame();
            CHECK(f.underlay.vertices.capacity() == capV);
            CHECK(f.underlay.strips.capacity() == capS);
            CHECK(f.underlay.segments.capacity() == capG);
            CHECK(f.overlay.segments.capacity() == capO);
        }
    }

    TEST_CASE("view culling drops the chords that cannot reach the view and keeps every chord that can")
    {
        // A circle of radius 500 m centred 500 m to the right of the camera: only the part near the camera is in a 300 m view.
        Trail circle;
        const double w = 2.0 * kPi / 3600.0;
        for (int i = 0; i <= 1440; ++i) {
            TrailSample s = circler(i * 2.5, 500.0, w, 0.0, 0.0);
            s.x += 500.0;
            circle.record(s, 0.0);
        }
        TrailDrawParams p;
        p.now = 1e30;
        p.offX = 0.0;
        p.offY = 0.0;
        p.metresPerPixel = 1.0;
        p.minStepPx = 0.0;
        p.colour = {1.0f, 1.0f, 1.0f, 1.0f};
        LineBatch full, culled;
        appendTrail(circle, nullptr, p, full);
        p.cullHalfW = 300.0;
        p.cullHalfH = 300.0;
        appendTrail(circle, nullptr, p, culled);
        REQUIRE(!full.strips.empty());
        REQUIRE(!culled.strips.empty());

        auto chordsOf = [](const LineBatch& b) {
            std::set<std::array<float, 4>> c;
            for (const LineStrip& s : b.strips)
                for (std::uint32_t k = 0; k + 1 < s.count; ++k) {
                    const LineVertex& a = b.vertices[s.first + k];
                    const LineVertex& e = b.vertices[s.first + k + 1];
                    c.insert({a.x, a.y, e.x, e.y});
                }
            return c;
        };
        const auto cf = chordsOf(full), cc = chordsOf(culled);
        CHECK(cc.size() < cf.size() / 2);                     // most of the loop is outside the view
        const float lim = 300.0f + 9.5f;                      // half view + margin (8 px) + the line width (1.5 px)
        for (const auto& c : cc) CHECK(cf.count(c) == 1);     // nothing invented
        for (const auto& c : cf) {
            const bool outside = (c[0] > lim && c[2] > lim) || (c[0] < -lim && c[2] < -lim) || (c[1] > lim && c[3] > lim) ||
                                 (c[1] < -lim && c[3] < -lim);
            if (!outside) CHECK(cc.count(c) == 1);            // every chord near the view survives
        }
        // A strip is cut where the trail leaves the view: a loop that passes through it twice makes two strips at least.
        CHECK(culled.strips.size() >= 1);
        // The fade does not depend on which chords are kept: the last vertex of the full trail is the last one of the culled
        // trail only if it is in view, but every kept vertex carries the alpha of the unculled one.
        std::set<std::array<float, 3>> va;
        for (const LineVertex& v : full.vertices) va.insert({v.x, v.y, v.a});
        for (const LineVertex& v : culled.vertices) CHECK(va.count({v.x, v.y, v.a}) == 1);
    }

    TEST_CASE("the geometry of trails and orbits does not depend on the number of worker threads")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        ThreadPool pool(4);
        auto run = [&](ThreadPool* tp) {
            OverlayBuilder ob;
            ob.setThreadPool(tp);
            ob.settings.trails = true;
            ob.settings.osculatingOrbits = true;
            RenderFrame f = frameOf(2000, 1200);
            SimDriver dd(w.res, SceneSettings{});
            for (int i = 1; i <= 60; ++i) {
                dd.advanceTo(i * 4000.0);
                f.clearObjects();
                OverlayView v = viewAt(bodyOf(dd.state(), w.planet)->r, 1e6);
                ob.build(dd.state(), dd.sim(), 1, v, f);
            }
            return f;
        };
        const RenderFrame a = run(nullptr), b = run(&pool);
        REQUIRE(a.underlay.vertices.size() > 100);
        REQUIRE(a.underlay.vertices.size() == b.underlay.vertices.size());
        REQUIRE(a.underlay.strips.size() == b.underlay.strips.size());
        REQUIRE(a.underlay.segments.size() == b.underlay.segments.size());
        CHECK(std::memcmp(a.underlay.vertices.data(), b.underlay.vertices.data(), a.underlay.vertices.size() * sizeof(LineVertex)) == 0);
        for (std::size_t k = 0; k < a.underlay.strips.size(); ++k) {
            CHECK(a.underlay.strips[k].first == b.underlay.strips[k].first);
            CHECK(a.underlay.strips[k].count == b.underlay.strips[k].count);
        }
    }
}
