#include <doctest.h>
#include <cmath>
#include "test_app_overlays_util.h"

using namespace ovl_test;

TEST_SUITE("app overlays arrows")
{
    TEST_CASE("the three scaling modes")
    {
        CHECK(arrowShape(ArrowScale::Linear, 0.25) == doctest::Approx(0.25));
        CHECK(arrowShape(ArrowScale::Sqrt, 0.25) == doctest::Approx(0.5));
        CHECK(arrowShape(ArrowScale::Log, 1.0) == doctest::Approx(1.0));
        CHECK(arrowShape(ArrowScale::Log, 1e-3) == doctest::Approx(0.5));    // three of six decades down
        CHECK(arrowShape(ArrowScale::Log, 1e-9) == 0.0);                     // beyond the range: nothing
        for (ArrowScale m : {ArrowScale::Linear, ArrowScale::Sqrt, ArrowScale::Log}) {
            CHECK(arrowShape(m, 0.0) == 0.0);
            CHECK(arrowShape(m, -1.0) == 0.0);
            CHECK(arrowShape(m, std::nan("")) == 0.0);
        }
        // Length = slider * shape(magnitude / reference), at most kArrowMaxFactor sliders long.
        CHECK(arrowLengthPx(ArrowScale::Linear, 50.0, 100.0, 80.0) == doctest::Approx(40.0));
        CHECK(arrowLengthPx(ArrowScale::Sqrt, 25.0, 100.0, 80.0) == doctest::Approx(40.0));
        CHECK(arrowLengthPx(ArrowScale::Linear, 1e9, 100.0, 80.0) == doctest::Approx(kArrowMaxFactor * 80.0));
        CHECK(arrowLengthPx(ArrowScale::Linear, 50.0, 0.0, 80.0) == 0.0);    // no reference, no arrow
        CHECK(arrowLengthPx(ArrowScale::Linear, 50.0, 100.0, 0.0) == 0.0);
    }

    TEST_CASE("sqrt and log keep small arrows visible where linear loses them")
    {
        const double lin = arrowLengthPx(ArrowScale::Linear, 1e-3, 1.0, 100.0);
        const double sq = arrowLengthPx(ArrowScale::Sqrt, 1e-3, 1.0, 100.0);
        const double lg = arrowLengthPx(ArrowScale::Log, 1e-3, 1.0, 100.0);
        CHECK(lin < kArrowMinPx);
        CHECK(sq > lin);
        CHECK(lg > sq);
    }

    TEST_CASE("a glyph is a shaft of the right length and a two-barb head")
    {
        LineBatch b;
        REQUIRE(appendArrow(b, 10.0, 20.0, 0.0, 3.0, 50.0, 100.0, Rgba{1, 0, 0, 1}, 2.0f));
        REQUIRE(b.segments.size() == 3);
        const LineSegment& shaft = b.segments[0];
        CHECK(shaft.a.x == doctest::Approx(10.0f));
        CHECK(shaft.a.y == doctest::Approx(20.0f));
        CHECK(shaft.b.x == doctest::Approx(10.0f));                 // the direction was normalised (0, 3) -> (0, 1)
        CHECK(shaft.b.y == doctest::Approx(20.0f + 50.0f * 100.0f)); // 50 px at 100 m/px
        CHECK(shaft.widthPx == 2.0f);
        for (int i = 1; i <= 2; ++i) {
            const LineSegment& barb = b.segments[static_cast<std::size_t>(i)];
            CHECK(barb.a.x == shaft.b.x);                            // both barbs start at the tip
            CHECK(barb.a.y == shaft.b.y);
            CHECK(barb.b.y < shaft.b.y);                             // and point back towards the tail
            const double len = std::hypot(barb.b.x - barb.a.x, barb.b.y - barb.a.y) / 100.0;
            CHECK(len == doctest::Approx(12.0).epsilon(1e-4));       // 30% of 50 px = 15, capped at 12
        }
        CHECK(b.segments[1].b.x == doctest::Approx(-b.segments[2].b.x + 20.0f).epsilon(1e-5));   // symmetric about the shaft
        // Too short, or no direction: nothing is added.
        LineBatch c;
        CHECK_FALSE(appendArrow(c, 0, 0, 1, 0, 1.0, 1.0, Rgba{}));
        CHECK_FALSE(appendArrow(c, 0, 0, 0, 0, 50.0, 1.0, Rgba{}));
        CHECK(c.empty());
    }

    // The shaft (first segment of a glyph) whose tail is at p (camera-relative), or null.
    static const LineSegment* shaftAt(const LineBatch& b, const Vec3& p, double tol)
    {
        for (std::size_t i = 0; i < b.segments.size(); i += 3)
            if (std::hypot(b.segments[i].a.x - p.x, b.segments[i].a.y - p.y) < tol) return &b.segments[i];
        return nullptr;
    }

    TEST_CASE("velocity arrows follow the selected reference frame")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        d.advanceTo(1000.0);
        const DisplayState& st = d.state();
        const DisplayBody& star = *bodyOf(st, w.star);
        const DisplayBody& planet = *bodyOf(st, w.planet);
        const DisplayBody& moon = *bodyOf(st, w.moon);

        OverlayBuilder ob;
        ob.settings.velocityArrows = true;
        ob.settings.arrowMode = ArrowScale::Linear;
        ob.settings.arrowLengthPx = 100.0;
        ob.settings.velocityReference = 40000.0;      // m/s: the planet's 29.8 km/s is 0.745 of it
        const double mpp = 1e7;
        const double tol = 1e5;                       // float rounding of ~1.5e11 m
        auto lengthPx = [&](const LineSegment* s) { return std::hypot(s->b.x - s->a.x, s->b.y - s->a.y) / mpp; };

        // Inertial frame: the planet's arrow is its inertial velocity (29.8 km/s -> 74.5 px), the star's is too short to draw.
        {
            RenderFrame f = frameOf();
            ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, mpp), f);
            CHECK(f.underlay.empty());
            const LineSegment* sp = shaftAt(f.overlay, planet.r, tol);
            REQUIRE(sp != nullptr);
            CHECK(lengthPx(sp) == doctest::Approx(100.0 * std::hypot(planet.v.x, planet.v.y) / 40000.0).epsilon(1e-4));
            CHECK(shaftAt(f.overlay, star.r, tol) == nullptr);
            CHECK(ob.stats().arrows == 2);            // planet and moon
        }
        // Follow-body frame of the planet: its own arrow vanishes, the star runs at -v_planet, the moon at its orbital speed.
        {
            OverlayView v = viewAt(planet.r, mpp);
            v.frameMode = FrameMode::FollowBody;
            v.frameBody = w.planet;
            RenderFrame f = frameOf();
            ob.build(d.state(), d.sim(), 1, v, f);
            CHECK(shaftAt(f.overlay, Vec3{0, 0, 0}, tol) == nullptr);   // the planet sits at the centre and does not move
            const LineSegment* ss = shaftAt(f.overlay, Vec3{star.r.x - planet.r.x, star.r.y - planet.r.y, 0}, tol);
            REQUIRE(ss != nullptr);
            CHECK(lengthPx(ss) == doctest::Approx(100.0 * std::hypot(star.v.x - planet.v.x, star.v.y - planet.v.y) / 40000.0).epsilon(1e-3));
            // The arrow points along -v_planet (to within the star's own tiny velocity).
            CHECK((ss->b.x - ss->a.x) * planet.v.x + (ss->b.y - ss->a.y) * planet.v.y < 0.0);
            const LineSegment* sm = shaftAt(f.overlay, Vec3{moon.r.x - planet.r.x, moon.r.y - planet.r.y, 0}, tol);
            REQUIRE(sm != nullptr);
            const double vRel = std::hypot(moon.v.x - planet.v.x, moon.v.y - planet.v.y);
            CHECK(vRel == doctest::Approx(1018.0).epsilon(0.02));
            CHECK(lengthPx(sm) == doctest::Approx(100.0 * vRel / 40000.0).epsilon(1e-3));
        }
        // Sqrt mode with the automatic reference: the fastest arrow gets exactly the slider length.
        {
            ob.settings.arrowMode = ArrowScale::Sqrt;
            ob.settings.velocityReference = 0.0;
            RenderFrame f = frameOf();
            ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, mpp), f);
            const LineSegment* sp = shaftAt(f.overlay, planet.r, tol);
            REQUIRE(sp != nullptr);
            const LineSegment* sm = shaftAt(f.overlay, moon.r, tol);
            REQUIRE(sm != nullptr);
            // The moon (planet + orbital speed) is the fastest object: it gets the full slider length.
            CHECK(lengthPx(sm) == doctest::Approx(100.0).epsilon(1e-4));
            const double ratio = std::hypot(planet.v.x, planet.v.y) / std::hypot(moon.v.x, moon.v.y);
            CHECK(lengthPx(sp) == doctest::Approx(100.0 * std::sqrt(ratio)).epsilon(1e-3));
        }
    }

    TEST_CASE("acceleration arrows point at the heavy body and shuttle arrows include thrust")
    {
        ResolvedScene res = driver_test::lucernaWithShuttles(2);   // shuttle 900 coasts, 901 is a Prograde autopilot
        SimDriver d(res, SceneSettings{});
        d.advanceTo(30.0);
        const DisplayState& st = d.state();
        OverlayBuilder ob;
        ob.settings.accelerationArrows = true;
        ob.settings.onBodies = false;
        ob.settings.arrowMode = ArrowScale::Linear;
        ob.settings.arrowLengthPx = 100.0;
        ob.settings.accelerationReference = 10.0;
        RenderFrame f = frameOf();
        ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e5), f);
        REQUIRE(ob.stats().arrows == 2);
        const DisplayBody* domus = nullptr;
        for (const DisplayBody& b : st.bodies)
            if (b.name == "Domus") domus = &b;
        REQUIRE(domus != nullptr);
        const DisplayShuttle& coast = st.shuttles[0];
        const LineSegment* s = shaftAt(f.overlay, coast.r, 1e5);
        REQUIRE(s != nullptr);
        // Gravity of Domus dominates the coasting shuttle: its arrow points to Domus' centre.
        const double ax = s->b.x - s->a.x, ay = s->b.y - s->a.y;
        const double tx = domus->r.x - coast.r.x, ty = domus->r.y - coast.r.y;
        CHECK((ax * tx + ay * ty) / (std::hypot(ax, ay) * std::hypot(tx, ty)) > 0.999);
        const double g = domus->mu / (std::hypot(tx, ty) * std::hypot(tx, ty));
        CHECK(std::hypot(ax, ay) / 1e5 == doctest::Approx(100.0 * g / 10.0).epsilon(0.02));
    }
}
