#include <doctest.h>
#include "app/camera.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace cosmos;
using namespace cosmos::app;

namespace {

PickCandidate body(std::uint64_t id, float x, float y, float radius)
{
    PickCandidate c;
    c.kind = TargetKind::Body;
    c.id = id;
    c.x = x;
    c.y = y;
    c.radius = radius;
    return c;
}

PickCandidate shuttle(std::uint64_t id, float x, float y)
{
    PickCandidate c = body(id, x, y, 1.0f);
    c.kind = TargetKind::Shuttle;
    return c;
}

} // namespace

TEST_SUITE("app_camera_pick")
{
    TEST_CASE("empty candidate list and nothing in reach give no hit")
    {
        CHECK_FALSE(pickNearest({}, 0.0, 0.0, 1.0).hit);
        const std::vector<PickCandidate> c = {body(1, 100.0f, 0.0f, 5.0f)};
        const PickResult r = pickNearest(c, 0.0, 0.0, 1.0);    // 100 px away, radius 5 px -> pick radius 8 px
        CHECK_FALSE(r.hit);
        CHECK(r.id == 0);
        CHECK(r.kind == TargetKind::None);
        CHECK_FALSE(pickNearest(c, 0.0, 0.0, 0.0).hit);        // invalid scale
    }

    TEST_CASE("the minimum pick radius is 8 px: tiny objects can still be clicked")
    {
        // A 2 m shuttle at 1e6 m/px is 2e-6 px across; it must still be pickable within 8 px, not at 8.01.
        const double mpp = 1e6;
        const std::vector<PickCandidate> c = {shuttle(7, 0.0f, 0.0f)};
        auto at = [&](double pxRight) { return pickNearest(c, pxRight * mpp, 0.0, mpp); };
        CHECK(at(0.0).hit);
        CHECK(at(7.9).hit);
        CHECK(at(-7.9).hit);
        CHECK_FALSE(at(8.1).hit);
        CHECK(at(7.9).id == 7);
        CHECK(at(7.9).kind == TargetKind::Shuttle);
        CHECK(at(7.9).distancePx == doctest::Approx(7.9).epsilon(1e-6));
        // the radius is circular, not a box
        CHECK_FALSE(pickNearest(c, 6.0 * mpp, 6.0 * mpp, mpp).hit);     // 8.49 px
        CHECK(pickNearest(c, 5.0 * mpp, 5.0 * mpp, mpp).hit);           // 7.07 px
        // a different minimum
        CHECK_FALSE(pickNearest(c, 7.9 * mpp, 0.0, mpp, 4.0).hit);
    }

    TEST_CASE("a large body is picked anywhere on its disc, not just near its centre")
    {
        const double mpp = 1000.0;
        const std::vector<PickCandidate> c = {body(1, 0.0f, 0.0f, 100000.0f)};   // radius 100 px
        CHECK(pickNearest(c, 99.0 * mpp, 0.0, mpp).hit);
        CHECK(pickNearest(c, 0.0, -70.0 * mpp, mpp).hit);
        CHECK_FALSE(pickNearest(c, 101.0 * mpp, 0.0, mpp).hit);
        CHECK_FALSE(pickNearest(c, 80.0 * mpp, 80.0 * mpp, mpp).hit);   // 113 px
        // the pick radius follows the zoom: at 100x the scale the same body is 1 px, so the 8 px minimum applies
        CHECK(pickNearest(c, 7.0 * mpp * 100.0, 0.0, mpp * 100.0).hit);
        CHECK_FALSE(pickNearest(c, 9.0 * mpp * 100.0, 0.0, mpp * 100.0).hit);
    }

    TEST_CASE("the nearest centre wins when several pick discs contain the cursor")
    {
        const double mpp = 1.0;
        // a planet (radius 200 px) with a moon (radius 3 px) inside its disc and a shuttle beside the moon
        const std::vector<PickCandidate> c = {body(1, 0.0f, 0.0f, 200.0f), body(2, 120.0f, 0.0f, 3.0f),
                                              shuttle(30, 126.0f, 0.0f)};
        CHECK(pickNearest(c, 119.0, 0.0, mpp).id == 2);         // 1 px from the moon
        CHECK(pickNearest(c, 124.5, 0.0, mpp).id == 30);        // 1.5 px from the shuttle, 4.5 from the moon
        CHECK(pickNearest(c, 40.0, 0.0, mpp).id == 1);          // the moon is 80 px away: outside its 8 px reach
        CHECK(pickNearest(c, 50.0, 30.0, mpp).id == 1);
        CHECK_FALSE(pickNearest(c, 300.0, 0.0, mpp).hit);
    }

    TEST_CASE("ties do not depend on candidate order")
    {
        const double mpp = 1.0;
        std::vector<PickCandidate> c = {body(5, -10.0f, 0.0f, 30.0f), body(3, 10.0f, 0.0f, 30.0f),
                                        body(9, 10.0f, 0.0f, 12.0f), body(4, 10.0f, 0.0f, 12.0f)};
        // cursor at x = 0 is 10 px from all four; the smaller pick radius wins, then the smaller id
        std::sort(c.begin(), c.end(), [](const PickCandidate& a, const PickCandidate& b) { return a.id < b.id; });
        int perms = 0;
        do {
            const PickResult r = pickNearest(c, 0.0, 0.0, mpp);
            REQUIRE(r.hit);
            CHECK(r.id == 4);
            ++perms;
        } while (std::next_permutation(c.begin(), c.end(),
                                       [](const PickCandidate& a, const PickCandidate& b) { return a.id < b.id; }));
        CHECK(perms == 24);
    }

    TEST_CASE("non-finite candidates are never hit")
    {
        const std::vector<PickCandidate> c = {body(1, std::nanf(""), 0.0f, 10.0f), body(2, 0.0f, 0.0f, 1.0f)};
        CHECK(pickNearest(c, 0.0, 0.0, 1.0).id == 2);
    }

    TEST_CASE("picking through the camera at 1e11 m: window click to camera-relative to id")
    {
        // A star with a planet 1.5e11 m away and a shuttle 400 m from the planet, camera far from the origin.
        const Vec3 star{3.0e11, -1.0e11, 0.0};
        const Vec3 planet{star.x + 1.496e11, star.y + 2.0e10, 0.0};
        const Vec3 craft{planet.x + 300.0, planet.y - 200.0, 0.0};
        Camera cam;
        cam.setCentreWorld(planet);
        cam.setMetresPerPixel(50.0);        // the shuttle is 360 m = 7.2 px from the planet's centre
        const Viewport vp{1920, 1080};

        auto candidates = [&]() {
            std::vector<PickCandidate> v;
            const Camera::Rel s = cam.toCameraRelative(star), p = cam.toCameraRelative(planet),
                              h = cam.toCameraRelative(craft);
            v.push_back(body(1, s.x, s.y, 6.96e8f));
            v.push_back(body(2, p.x, p.y, 6.4e6f));
            v.push_back(shuttle(10, h.x, h.y));
            return v;
        };
        auto pickAt = [&](double wx, double wy) {
            double rx = 0, ry = 0;
            cam.windowToRelative(wx, wy, vp, rx, ry);
            return pickNearest(candidates(), rx, ry, cam.metresPerPixel());
        };

        // the planet's disc is 6.4e6 / 50 = 128000 px wide: it fills the screen; the shuttle is on top of it.
        // Click on the shuttle's screen position (window y is down, world y up).
        double sx = 0, sy = 0;
        cam.worldToWindow(craft, vp, sx, sy);
        CHECK(sx == doctest::Approx(960.0 + 6.0));
        CHECK(sy == doctest::Approx(540.0 + 4.0));
        CHECK(pickAt(sx, sy).id == 10);
        CHECK(pickAt(sx + 3.0, sy - 3.0).id == 10);
        CHECK(pickAt(sx + 0.0, sy + 100.0).id == 2);        // far from the shuttle: the planet disc
        CHECK(pickAt(960.0, 540.0).id == 2);                // dead centre: planet centre is nearer than the shuttle

        // zoom far out about the cursor: both collapse to markers, still pickable at 8 px
        cam.setMetresPerPixel(1e9);
        cam.worldToWindow(planet, vp, sx, sy);
        const PickResult r = pickAt(sx + 5.0, sy + 5.0);
        REQUIRE(r.hit);
        CHECK((r.id == 2 || r.id == 10));
        CHECK(pickAt(sx, sy).id == 2);                  
    }

    TEST_CASE("minimum-size marker rule: a body under 3 px across is a marker")
    {
        CHECK(isMarkerSized(1.0, 1.0));                         // 2 px
        CHECK(isMarkerSized(1.4999, 1.0));                      // 2.9998 px
        CHECK_FALSE(isMarkerSized(1.5, 1.0));                   // exactly 3 px is a disc
        CHECK_FALSE(isMarkerSized(100.0, 1.0));
        CHECK(isMarkerSized(6.371e6, 1e7));                     // Earth at 10000 km/px
        CHECK_FALSE(isMarkerSized(6.371e6, 1e6));
        CHECK(isMarkerSized(1.0, 1.0, 4.0));                    // a custom threshold
        CHECK_FALSE(isMarkerSized(2.0, 1.0, 4.0));
        // the on-screen radius is never below half the marker size
        CHECK(drawnRadiusPx(1.0, 1000.0) == 1.5);
        CHECK(drawnRadiusPx(10000.0, 1000.0) == doctest::Approx(10.0));
        CHECK(drawnRadiusPx(1.5, 1.0) == doctest::Approx(1.5));
        // the rule agrees with the renderer's default (RenderFrame::bodyMinPixels is 3)
        CHECK(kMarkerDiameterPx == 3.0);
        CHECK(kMinPickRadiusPx == 8.0);
    }
}
