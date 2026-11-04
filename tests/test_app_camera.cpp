#include <doctest.h>
#include "app/camera.h"
#include <algorithm>
#include <cfloat>
#include <cmath>

using namespace cosmos;
using namespace cosmos::app;

namespace {

const Viewport kVp{1600, 900};

// Window point under which `world` sits, for comparing before/after a zoom.
void windowOf(const Camera& c, const Vec3& w, double& px, double& py) { c.worldToWindow(w, kVp, px, py); }

} // namespace

TEST_SUITE("app_camera")
{
    TEST_CASE("window and centred mapping: centre is the viewport centre, y is flipped")
    {
        Camera cam;
        cam.setMetresPerPixel(100.0);
        cam.setCentreWorld({1000.0, 2000.0, 0.0});
        double px = 0, py = 0;
        cam.worldToWindow({1000.0, 2000.0, 0.0}, kVp, px, py);
        CHECK(px == doctest::Approx(800.0));
        CHECK(py == doctest::Approx(450.0));
        // 100 px right and 50 px up on screen (= smaller window y) is +10 km in x and +5 km in y
        cam.worldToWindow({1000.0 + 100.0 * 100.0, 2000.0 + 50.0 * 100.0, 0.0}, kVp, px, py);
        CHECK(px == doctest::Approx(900.0));
        CHECK(py == doctest::Approx(400.0));
        cam.worldToCentred({1000.0 - 300.0, 2000.0 - 200.0, 0.0}, px, py);
        CHECK(px == doctest::Approx(-3.0));
        CHECK(py == doctest::Approx(-2.0));
        const Vec3 w = cam.windowToWorld(0.0, 0.0, kVp);    // top-left corner
        CHECK(w.x == doctest::Approx(1000.0 - 800.0 * 100.0));
        CHECK(w.y == doctest::Approx(2000.0 + 450.0 * 100.0));
    }

    TEST_CASE("screen <-> world round trip at 1e11 m offsets is sub-pixel at every zoom")
    {
        // Rounding the world point to double costs at most half an ulp of the centre (1.5e-5 m at 1e11 m), and the
        // way back one more. In pixels that is ~ulp/mpp (measured worst 0.049 px at |centre| 7.8e11 m and 1 mm/px,
        // 0.012 px at 1.5e11 m). Bound it by one epsilon of the centre magnitude, plus a hair for the division.
        const Vec3 centres[] = {{1.496e11, -3.3e11, 0.0}, {-7.8e11, 5.9e11, 0.0}, {0.0, 0.0, 0.0}};
        const double mpps[] = {1e-3, 0.05, 1.0, 1e3, 1e6, 1e9, 1e12};
        double worst = 0.0;
        for (const Vec3& c : centres) {
            for (double mpp : mpps) {
                Camera cam;
                cam.setMetresPerPixel(mpp);
                cam.setCentreWorld(c);
                const double mag = std::max(std::fabs(c.x), std::fabs(c.y));
                const double bound = DBL_EPSILON * mag / mpp + 1e-9;
                for (double sx : {0.0, 1.0, 799.5, 1600.0}) {
                    for (double sy : {0.0, 123.25, 450.0, 900.0}) {
                        const Vec3 w = cam.windowToWorld(sx, sy, kVp);
                        double bx = 0, by = 0;
                        cam.worldToWindow(w, kVp, bx, by);
                        const double err = std::max(std::fabs(bx - sx), std::fabs(by - sy));
                        CHECK(err <= bound);
                        worst = std::max(worst, err);
                    }
                }
            }
        }
        CHECK(worst < 0.1);     // sub-pixel everywhere (worst: |centre| 7.8e11 m at 1 mm/px, ulp/2 = 6e-5 m = 0.06 px)
    }

    TEST_CASE("camera-relative conversion has no jitter at 1e11 m (plan 9.3)")
    {
        Camera cam;
        cam.setMetresPerPixel(1.0);
        const Vec3 centre{1.5e11 + 0.123, -2.0e11 + 0.456, 0.0};
        cam.setCentreWorld(centre);
        double worstCamera = 0.0, worstNaive = 0.0;
        for (int i = -50; i <= 50; ++i) {
            // objects within +-50 m of the camera (a station and its shuttles), 1 m apart
            const Vec3 p{centre.x + i * 1.0, centre.y - i * 0.5, 0.0};
            const Camera::Rel r = cam.toCameraRelative(p);
            worstCamera = std::max({worstCamera, std::fabs(r.x - i * 1.0), std::fabs(r.y + i * 0.5)});
            // the naive way: cast to float first, subtract afterwards
            const float nx = static_cast<float>(p.x) - static_cast<float>(centre.x);
            const float ny = static_cast<float>(p.y) - static_cast<float>(centre.y);
            worstNaive = std::max({worstNaive, std::fabs(nx - i * 1.0), std::fabs(ny + i * 0.5)});
        }
        // The double subtraction is exact to ulp(1e11) = 1.5e-5 m; casting 50 m to float adds ~4e-6 m.
        CHECK(worstCamera < 3e-5);
        // Casting first rounds every point to a multiple of 8192 m (float ulp at 1e11), so the 100 m of structure
        // collapses to 0 (error up to 50 m) or to a jump of 8192 m: millions of times worse.
        CHECK(worstNaive > 40.0);
    }

    TEST_CASE("zoom about the cursor keeps the point under the cursor fixed")
    {
        const Vec3 centres[] = {{0.0, 0.0, 0.0}, {1.496e11, 7.0e10, 0.0}};
        const double starts[] = {1e-2, 1.0, 1e5, 1e10};
        const double cursors[][2] = {{800, 450}, {0, 0}, {1600, 900}, {1234.5, 77.25}, {10, 880}};
        for (const Vec3& c : centres) {
            for (double mpp0 : starts) {
                for (const auto& cur : cursors) {
                    for (double steps : {1.0, -1.0, 3.0, -7.0, 0.5}) {
                        Camera cam;
                        cam.setMetresPerPixel(mpp0);
                        cam.setCentreWorld(c);
                        const Vec3 under = cam.windowToWorld(cur[0], cur[1], kVp);
                        const double before = cam.metresPerPixel();
                        CHECK(cam.zoomAtWindow(steps, cur[0], cur[1], kVp));
                        CHECK(cam.metresPerPixel() != before);
                        double px = 0, py = 0;
                        windowOf(cam, under, px, py);
                        // ulp of the world point (<= 1.5e-5 m at 1e11 m) divided by the new scale, in pixels (measured 4e-4 px at 1.5e11 m)
                        const double mag = std::max(std::fabs(under.x), std::fabs(under.y));
                        const double bound = DBL_EPSILON * mag / cam.metresPerPixel() + 1e-9;
                        CHECK(std::fabs(px - cur[0]) <= bound);
                        CHECK(std::fabs(py - cur[1]) <= bound);
                    }
                }
            }
        }
    }

    TEST_CASE("one wheel notch scales metres per pixel by the step factor, and zooming in then out returns")
    {
        Camera cam;
        cam.setMetresPerPixel(1000.0);
        cam.zoomAtCentred(1.0, 0.0, 0.0);
        CHECK(cam.metresPerPixel() == doctest::Approx(1000.0 / kZoomStepFactor).epsilon(1e-12));
        cam.zoomAtCentred(-1.0, 0.0, 0.0);
        CHECK(cam.metresPerPixel() == doctest::Approx(1000.0).epsilon(1e-12));
        // zoomLog is the natural log of metres per pixel (plan 9.3)
        CHECK(std::exp(cam.zoomLog()) == doctest::Approx(cam.metresPerPixel()).epsilon(1e-12));
        // zooming at the centre never moves the camera
        CHECK(cam.centreWorld().x == 0.0);
        CHECK(cam.centreWorld().y == 0.0);
    }

    TEST_CASE("zoom clamps to [1e-3, 1e12] m/px exactly and reports no change at the limit")
    {
        Camera cam;
        cam.setCentreWorld({5e10, -4e10, 0.0});
        for (int i = 0; i < 400; ++i) cam.zoomAtWindow(1.0, 100.0, 100.0, kVp);
        CHECK(cam.metresPerPixel() == kMinMetresPerPixel);
        const Vec3 atLimit = cam.centreWorld();
        CHECK_FALSE(cam.zoomAtWindow(1.0, 100.0, 100.0, kVp));      // already at the limit: nothing changes
        CHECK(cam.centreWorld().x == atLimit.x);
        CHECK(cam.centreWorld().y == atLimit.y);
        CHECK(cam.metresPerPixel() == kMinMetresPerPixel);

        for (int i = 0; i < 400; ++i) cam.zoomAtWindow(-1.0, 1500.0, 800.0, kVp);
        CHECK(cam.metresPerPixel() == kMaxMetresPerPixel);
        CHECK_FALSE(cam.zoomAtWindow(-1.0, 1500.0, 800.0, kVp));

        cam.setMetresPerPixel(1e-9);
        CHECK(cam.metresPerPixel() == kMinMetresPerPixel);
        cam.setMetresPerPixel(1e20);
        CHECK(cam.metresPerPixel() == kMaxMetresPerPixel);
        cam.setZoomLog(-1e6);
        CHECK(cam.metresPerPixel() == kMinMetresPerPixel);
        cam.setMetresPerPixel(42.0);
        cam.setMetresPerPixel(-3.0);                                // invalid input is ignored
        cam.setMetresPerPixel(std::nan(""));
        cam.setZoomLog(std::nan(""));
        CHECK(cam.metresPerPixel() == doctest::Approx(42.0).epsilon(1e-12));
        // the whole range is 15 decades: about 155 notches at 1.25x
        CHECK(std::log(kMaxMetresPerPixel / kMinMetresPerPixel) / std::log(kZoomStepFactor) == doctest::Approx(155.0).epsilon(0.01));
    }

    TEST_CASE("zoom at the clamp edge: a partial step still keeps the cursor point fixed")
    {
        Camera cam;
        cam.setMetresPerPixel(kMinMetresPerPixel * 1.1);            // less than one notch above the limit
        cam.setCentreWorld({3e9, 3e9, 0.0});
        const Vec3 under = cam.windowToWorld(1300.0, 200.0, kVp);
        CHECK(cam.zoomAtWindow(1.0, 1300.0, 200.0, kVp));
        CHECK(cam.metresPerPixel() == kMinMetresPerPixel);
        double px = 0, py = 0;
        windowOf(cam, under, px, py);
        CHECK(std::fabs(px - 1300.0) < 1e-3);
        CHECK(std::fabs(py - 200.0) < 1e-3);
    }

    TEST_CASE("pan: the world point under the cursor follows the cursor")
    {
        Camera cam;
        cam.setMetresPerPixel(2.5e5);
        cam.setCentreWorld({1.2e11, 3.4e10, 0.0});
        const Vec3 grabbed = cam.windowToWorld(700.0, 300.0, kVp);
        cam.panByWindowPixels(+120.0, -45.0);                       // drag right and up
        double px = 0, py = 0;
        windowOf(cam, grabbed, px, py);
        CHECK(px == doctest::Approx(820.0).epsilon(1e-9));
        CHECK(py == doctest::Approx(255.0).epsilon(1e-9));
        // dragging right moves the camera left (-x), dragging down moves it up (+y)
        cam.setCentreWorld({0.0, 0.0, 0.0});
        cam.panByWindowPixels(10.0, 0.0);
        CHECK(cam.centreWorld().x == doctest::Approx(-10.0 * 2.5e5));
        cam.panByWindowPixels(0.0, 10.0);
        CHECK(cam.centreWorld().y == doctest::Approx(10.0 * 2.5e5));
        cam.panWorld(1.0, 2.0);
        CHECK(cam.centreWorld().x == doctest::Approx(-10.0 * 2.5e5 + 1.0));
        cam.panWorld(std::nan(""), 1.0);                            // ignored
        CHECK(cam.centreWorld().y == doctest::Approx(10.0 * 2.5e5 + 2.0));
    }

    TEST_CASE("update without a target keeps the centre in the inertial frame")
    {
        Camera cam;
        cam.setCentreWorld({1e11, 2e11, 0.0});
        CameraSources none;
        cam.update(1234.5, none);
        CHECK(cam.centreWorld().x == 1e11);
        CHECK(cam.centreWorld().y == 2e11);
        cam.update(1e9, none);
        CHECK(cam.centreWorld().x == 1e11);
    }
}
