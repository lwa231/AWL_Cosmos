#include <doctest.h>
#include <cmath>
#include "app/camera3d.h"

using namespace cosmos;
using namespace cosmos::app;

TEST_CASE("view3d: the default view is flat and matches the 2D mapping")
{
    View3D v;
    CHECK(v.isFlat());
    CHECK(v.isTopDown());
    const double mpp = 1.0e6;
    const Projected p = v.project({3.0e8, -2.0e8, 0.0}, mpp, 720);
    CHECK(p.visible);
    CHECK(p.x == doctest::Approx(300.0));
    CHECK(p.y == doctest::Approx(-200.0));
    // Perspective top-down: the plane through the look-at point keeps the same scale
    v.setProjection(Projection::Perspective);
    CHECK_FALSE(v.isFlat());
    const Projected q = v.project({3.0e8, -2.0e8, 0.0}, mpp, 720);
    CHECK(q.x == doctest::Approx(300.0));
    CHECK(q.y == doctest::Approx(-200.0));
    // A point raised towards the eye looks bigger and is nearer
    const Projected h = v.project({3.0e8, -2.0e8, 1.0e8}, mpp, 720);
    CHECK(h.x > q.x);
    CHECK(h.depth < q.depth);
    CHECK(h.pixelsPerMetre > q.pixelsPerMetre);
}

TEST_CASE("view3d: plane picking inverts the projection")
{
    for (int proj = 0; proj < 2; ++proj) {
        View3D v;
        v.setProjection(proj ? Projection::Perspective : Projection::Orthographic);
        v.setOrientation(0.4, 0.5);
        const double mpp = 2.5e5;
        const Vec3 world{1.0e8, -3.0e7, 0.0};
        const Projected p = v.project(world, mpp, 800);
        REQUIRE(p.visible);
        Vec3 hit;
        REQUIRE(v.pickPlane(p.x, p.y, mpp, 800, 0.0, hit));
        CHECK(hit.x == doctest::Approx(world.x).epsilon(1e-9));
        CHECK(hit.y == doctest::Approx(world.y).epsilon(1e-9));
        CHECK(std::fabs(hit.z) < 1e-3);
    }
}

TEST_CASE("view3d: behind the eye is not visible in perspective")
{
    View3D v;
    v.setProjection(Projection::Perspective);
    const double D = v.eyeDistance(1.0, 720);
    CHECK_FALSE(v.project({0.0, 0.0, 2.0 * D}, 1.0, 720).visible);
    CHECK(v.project({0.0, 0.0, 0.5 * D}, 1.0, 720).visible);
}

TEST_CASE("view3d: T animates to the tilt and back to top-down")
{
    View3D v;
    v.toggleTopDown(0.5);
    CHECK(v.animating());
    CHECK_FALSE(v.isFlat());
    for (int i = 0; i < 10; ++i) v.update(0.1);
    CHECK_FALSE(v.animating());
    CHECK(v.pitch() == doctest::Approx(View3D::kTiltDefaultPitch));
    v.orbitByPixels(100.0, 0.0);
    const double tiltedYaw = v.yaw();
    v.toggleTopDown(0.5);
    v.update(1.0);
    CHECK(v.isTopDown());
    CHECK(v.isFlat());
    v.toggleTopDown(0.0);   // instant: back to the remembered tilt
    CHECK(v.yaw() == doctest::Approx(tiltedYaw));
    // Pitch is clamped at top-down
    v.orbitByPixels(0.0, 1.0e6);
    CHECK(v.pitch() == doctest::Approx(View3D::kTopPitch));
}

TEST_CASE("view3d: free fly moves the eye and leaving fly mode resets it")
{
    View3D v;
    v.setFlying(true);
    v.fly(1.0, 0.0, 0.0, 0.5, 1000.0, 600);   // forward = -z from top-down, half a screen height
    CHECK(v.flyOffset().z == doctest::Approx(-0.5 * 1000.0 * 600.0));
    CHECK_FALSE(v.isFlat());
    v.setFlying(false);
    CHECK(v.isFlat());
    CHECK(angleDelta(3.0, -3.0) == doctest::Approx(2.0 * 3.14159265358979323846 - 6.0));
}
