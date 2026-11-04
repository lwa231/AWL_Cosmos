#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "test_app_overlays_util.h"

using namespace ovl_test;

namespace {

Elements make(double a, double e, double omega, bool retro, double M = 0.7)
{
    Elements el;
    el.a = a;
    el.e = e;
    el.omega = omega;
    el.M = M;
    el.retrograde = retro;
    return el;
}

// Elements -> state -> elements again, the way the overlay sees an orbit.
Elements throughState(const Elements& el, double mu)
{
    const RelState s = elementsToState(el, mu);
    return stateToElements(s.r, s.v, mu);
}

double cross2(const Vec3& a, const Vec3& b) { return a.x * b.y - a.y * b.x; }

// The strip whose bounding box is between minSpan and maxSpan wide (orbits of very different size tell apart that way).
const LineStrip* stripOfSpan(const LineBatch& b, double minSpan, double maxSpan)
{
    for (const LineStrip& s : b.strips) {
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
        for (std::uint32_t k = 0; k < s.count; ++k) {
            const LineVertex& v = b.vertices[s.first + k];
            x0 = std::min<double>(x0, v.x); x1 = std::max<double>(x1, v.x);
            y0 = std::min<double>(y0, v.y); y1 = std::max<double>(y1, v.y);
        }
        const double span = std::max(x1 - x0, y1 - y0);
        if (span >= minSpan && span <= maxSpan) return &s;
    }
    return nullptr;
}

} // namespace

TEST_SUITE("app overlays orbits")
{
    TEST_CASE("an ellipse has periapsis a(1-e) and apoapsis a(1+e), and closes")
    {
        const double mu = kPlanetMu, a = 1.2e7;
        for (double e : {0.0, 0.3, 0.9})
            for (bool retro : {false, true})
                for (double omega : {0.0, 1.1, -2.5}) {
                    const Elements el = throughState(make(a, e, omega, retro), mu);
                    OrbitCurve c;
                    REQUIRE(buildOrbitCurve(el, 0.0, 256, c));
                    REQUIRE(c.points.size() == 257);
                    CHECK(c.closed);
                    CHECK(c.hasApoapsis);
                    double dmin = 1e300, dmax = 0.0;
                    for (const Vec3& p : c.points) {
                        const double r = std::hypot(p.x, p.y);
                        dmin = std::min(dmin, r);
                        dmax = std::max(dmax, r);
                    }
                    CHECK(dmin == doctest::Approx(a * (1.0 - e)).epsilon(1e-9));
                    CHECK(dmax == doctest::Approx(a * (1.0 + e)).epsilon(1e-9));
                    CHECK(std::hypot(c.periapsis.x, c.periapsis.y) == doctest::Approx(a * (1.0 - e)).epsilon(1e-9));
                    CHECK(std::hypot(c.apoapsis.x, c.apoapsis.y) == doctest::Approx(a * (1.0 + e)).epsilon(1e-9));
                    CHECK(c.radiusMax == doctest::Approx(a * (1.0 + e)).epsilon(1e-9));
                    CHECK(c.points.front().x == c.points.back().x);
                    CHECK(c.points.front().y == c.points.back().y);
                    if (e > 0.0) {   // periapsis lies in the direction omega (the circle has no periapsis)
                        CHECK(std::atan2(c.periapsis.y, c.periapsis.x) == doctest::Approx(std::remainder(omega, 2.0 * kPi)).epsilon(1e-6));
                    }
                    // The polyline runs the way the body moves: counter-clockwise for h > 0, clockwise otherwise.
                    const Vec3 &p0 = c.points[128], &p1 = c.points[129];
                    CHECK((cross2(p0, p1) > 0.0) == !retro);
                    // And it passes through the body's own position.
                    const RelState s = elementsToState(make(a, e, omega, retro), mu);
                    double best = 1e300;   // distance to the polyline (a chord sags a little below the true curve)
                    for (std::size_t k = 0; k + 1 < c.points.size(); ++k) {
                        const Vec3 &p = c.points[k], &q = c.points[k + 1];
                        const double dx = q.x - p.x, dy = q.y - p.y, l2 = dx * dx + dy * dy;
                        const double u = l2 > 0.0 ? std::clamp(((s.r.x - p.x) * dx + (s.r.y - p.y) * dy) / l2, 0.0, 1.0) : 0.0;
                        best = std::min(best, std::hypot(p.x + u * dx - s.r.x, p.y + u * dy - s.r.y));
                    }
                    CHECK(best < 1.0e-3 * a * (1.0 + e));
                }
    }

    TEST_CASE("a hyperbola is drawn over the valid true anomaly only, up to the cap")
    {
        const double mu = kPlanetMu, q = 1.0e7, e = 2.0, a = -q / (e - 1.0);
        const Elements el = throughState(make(a, e, 0.4, false), mu);
        REQUIRE(el.e > 1.0);
        const double rCap = 100.0 * q;
        OrbitCurve c;
        REQUIRE(buildOrbitCurve(el, rCap, 256, c));
        CHECK_FALSE(c.closed);
        CHECK_FALSE(c.hasApoapsis);
        REQUIRE(c.points.size() == 257);
        double dmin = 1e300;
        const double nuInf = std::acos(-1.0 / e);
        for (const Vec3& p : c.points) {
            const double r = std::hypot(p.x, p.y);
            dmin = std::min(dmin, r);
            CHECK(r <= rCap * (1.0 + 1e-9));
            // The angle from the periapsis direction stays inside +-nu_inf (the mathematical other branch is not drawn).
            const double ang = std::remainder(std::atan2(p.y, p.x) - el.omega, 2.0 * kPi);
            CHECK(std::abs(ang) <= nuInf + 1e-9);
        }
        CHECK(dmin == doctest::Approx(q).epsilon(1e-9));
        CHECK(std::hypot(c.points.front().x, c.points.front().y) == doctest::Approx(rCap).epsilon(1e-9));
        CHECK(std::hypot(c.points.back().x, c.points.back().y) == doctest::Approx(rCap).epsilon(1e-9));
        // The two ends are on opposite sides of the periapsis line.
        CHECK(cross2(c.points.front(), c.points.back()) != 0.0);
    }

    TEST_CASE("degenerate orbits are refused")
    {
        OrbitCurve c;
        // A radial fall has h = 0, p = 0.
        const Elements radial = stateToElements({1e7, 0, 0}, {-3000.0, 0, 0}, kPlanetMu);
        CHECK_FALSE(buildOrbitCurve(radial, 1e9, 256, c));
        CHECK(c.points.empty());
        // A cap inside the periapsis leaves nothing of a hyperbola.
        const Elements hyp = throughState(make(-1e7, 2.0, 0.0, false), kPlanetMu);
        CHECK_FALSE(buildOrbitCurve(hyp, 0.5 * hyp.q, 256, c));
        Elements nan = make(1e7, 0.1, 0.0, false);
        nan.p = std::nan("");
        CHECK_FALSE(buildOrbitCurve(nan, 1e9, 256, c));
    }

    TEST_CASE("the builder draws the moon about the planet and the planet about the star")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        d.advanceTo(86400.0);
        const DisplayBody& star = *bodyOf(d.state(), w.star);
        const DisplayBody& planet = *bodyOf(d.state(), w.planet);
        const DisplayBody& moon = *bodyOf(d.state(), w.moon);

        OverlayBuilder ob;
        ob.settings.osculatingOrbits = true;
        RenderFrame f = frameOf();
        const double mpp = 3e8;                      // the planet orbit (1.5e11 m) is a 500 px radius loop through the view
        ob.build(d.state(), d.sim(), 1, viewAt(star.r, mpp), f);
        // The star has no parent. The moon's loop (4 px) is 1500 px from the camera, off screen, and is culled.
        CHECK(ob.stats().orbits == 1);
        CHECK(f.underlay.strips.size() == 1);
        CHECK(f.underlay.segments.size() == 4);      // periapsis '+' and apoapsis 'x' (two segments each)

        // Planet: relative to the star, mu = star + planet.
        {
            const Elements el = stateToElements(planet.r - star.r, planet.v - star.v, star.mu + planet.mu);
            const LineStrip* s = stripOfSpan(f.underlay, 1e11, 1e12);
            REQUIRE(s != nullptr);
            REQUIRE(s->count == 257);
            double dmin = 1e300, dmax = 0.0;
            for (std::uint32_t k = 0; k < s->count; ++k) {
                const LineVertex& v = f.underlay.vertices[s->first + k];
                const double r = std::hypot(v.x, v.y);   // the camera is at the star
                dmin = std::min(dmin, r);
                dmax = std::max(dmax, r);
            }
            CHECK(el.e < 0.01);
            CHECK(dmin == doctest::Approx(el.a * (1.0 - el.e)).epsilon(1e-5));   // periapsis distance a(1-e)
            CHECK(dmax == doctest::Approx(el.a * (1.0 + el.e)).epsilon(1e-5));
        }
        // Moon: its ellipse is centred on the planet, with the planet's mu: radius ~ 3.84e8 about the PLANET.
        {
            f.clearObjects();
            ob.build(d.state(), d.sim(), 1, viewAt(planet.r, 1e6), f);
            const Elements el = stateToElements(moon.r - planet.r, moon.v - planet.v, planet.mu + moon.mu);
            CHECK(el.a == doctest::Approx(kMoonA).epsilon(0.05));
            const LineStrip* s = stripOfSpan(f.underlay, 1e8, 2e9);
            REQUIRE(s != nullptr);
            double dmin = 1e300;
            for (std::uint32_t k = 0; k < s->count; ++k) {
                const LineVertex& v = f.underlay.vertices[s->first + k];
                dmin = std::min(dmin, static_cast<double>(std::hypot(v.x, v.y)));
            }
            CHECK(dmin == doctest::Approx(el.a * (1.0 - el.e)).epsilon(1e-5));
        }
    }

    TEST_CASE("a shuttle on an escape trajectory gets a hyperbola with the right periapsis")
    {
        SunPlanetMoon w;
        const BodyDesc planetD = [&] {
            BodyDesc p;
            p.position = {kPlanetA, 0.0, 0.0};
            p.velocity = {0.0, std::sqrt(kStarMu / kPlanetA), 0.0};
            return p;
        }();
        const double q = 7.0e6;
        const Vec3 rel{q, 0.0, 0.0}, vrel{0.0, 12000.0, 0.0};           // above the 10.6 km/s escape speed
        w.res.shuttles.push_back(sim_test::resolvedShuttle(900, planetD.position + rel, planetD.velocity + vrel));
        SimDriver d(w.res, SceneSettings{});
        const DisplayBody& planet = *bodyOf(d.state(), w.planet);
        REQUIRE(d.state().shuttles.size() == 1);
        const DisplayShuttle& sh = d.state().shuttles[0];

        OverlayBuilder ob;
        ob.settings.osculatingOrbits = true;
        ob.settings.onBodies = false;
        RenderFrame f = frameOf();
        ob.build(d.state(), d.sim(), 1, viewAt(planet.r, 1e5), f);
        REQUIRE(ob.stats().orbits == 1);
        const Elements el = stateToElements(sh.r - planet.r, sh.v - planet.v, planet.mu);
        REQUIRE(el.e > 1.0);
        const LineStrip& s = f.underlay.strips[0];
        double dmin = 1e300, dmaxv = 0.0;
        for (std::uint32_t k = 0; k < s.count; ++k) {
            const LineVertex& v = f.underlay.vertices[s.first + k];
            dmin = std::min(dmin, static_cast<double>(std::hypot(v.x, v.y)));
            dmaxv = std::max(dmaxv, static_cast<double>(std::hypot(v.x, v.y)));
        }
        CHECK(dmin == doctest::Approx(el.q).epsilon(1e-5));
        CHECK(dmin == doctest::Approx(el.a * (1.0 - el.e)).epsilon(1e-5));
        CHECK(dmaxv > 1280.0 * 1e5);                        // the arms leave the screen (viewport 1280 px at 1e5 m/px)
        CHECK(f.underlay.segments.size() == 2);             // periapsis mark only: no apoapsis on an open orbit
    }

    TEST_CASE("orbits that are a dot or entirely off screen are skipped")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        OverlayBuilder ob;
        ob.settings.osculatingOrbits = true;
        RenderFrame f = frameOf();
        ob.build(d.state(), d.sim(), 1, viewAt({0, 0, 0}, 1e12), f);          // every orbit is smaller than 1.5 px
        CHECK(ob.stats().orbits == 0);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, viewAt({1e15, 0, 0}, 1e6), f);        // far away from everything
        CHECK(ob.stats().orbits == 0);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, viewAt({kPlanetA, 0, 0}, 1e6), f);    // at the planet: the moon's orbit is visible
        CHECK(ob.stats().orbits >= 1);
    }

    TEST_CASE("a binary of equal masses: each member of the pair orbits the other, the pair orbits the star")
    {
        const double d = 2.0e7, muB = 1.0e15, ra = 1.5e11;
        const double vPair = std::sqrt(kStarMu / ra);
        const double vRel = std::sqrt(2.0 * muB / d);                      // circular relative speed of the pair
        BodyDesc s;
        s.mu = kStarMu;
        s.radius = 6.96e8;
        BodyDesc b1, b2;
        b1.mu = b2.mu = muB;
        b1.radius = b2.radius = 2.0e6;
        b1.position = {ra + 0.5 * d, 0.0, 0.0};
        b2.position = {ra - 0.5 * d, 0.0, 0.0};
        b1.velocity = {0.0, vPair + 0.5 * vRel, 0.0};
        b2.velocity = {0.0, vPair - 0.5 * vRel, 0.0};
        ResolvedScene res = sim_test::sceneOf({s, b1, b2});
        SimDriver drv(res, SceneSettings{});
        const DisplayBody& B1 = drv.state().bodies[1];

        OverlayBuilder ob;
        ob.settings.osculatingOrbits = true;
        RenderFrame f = frameOf();
        ob.build(drv.state(), drv.sim(), 1, viewAt(B1.r, 2e5), f);
        CHECK(ob.stats().orbits == 2);                                    // B2 about B1, and B1 about the star
        // B2's loop is centred on B1 (the camera): radius d, even though the star is much heavier.
        const LineStrip* s2 = stripOfSpan(f.underlay, 1e7, 1e8);   // B2's loop around B1 is 2d = 4e7 wide
        REQUIRE(s2 != nullptr);
        for (std::uint32_t k = 0; k < s2->count; ++k) {
            const LineVertex& v = f.underlay.vertices[s2->first + k];
            CHECK(std::hypot(v.x, v.y) == doctest::Approx(d).epsilon(2e-3));
        }
    }
}
