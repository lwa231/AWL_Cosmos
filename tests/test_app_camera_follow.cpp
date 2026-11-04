#include <doctest.h>
#include "app/camera.h"
#include "test_app_camera_util.h"
#include <cmath>

using namespace cosmos;
using namespace cosmos::app;
using namespace camtest;

namespace {

const Viewport kVp{1280, 720};

// Body 1 ("Domus"-like) orbits fast at 1.5e11 m; body 2 and 3 are other movers; shuttle 10 sits near body 1.
FakeWorld makeWorld()
{
    FakeWorld w;
    w.bodies[1] = {{1.5e11, 2.0e10, 0.0}, {3.0e4, -1.0e4, 0.0}};
    w.bodies[2] = {{-4.0e10, 9.0e10, 0.0}, {-5.0e3, 2.0e3, 0.0}};
    w.bodies[3] = {{7.0e10, -6.0e10, 0.0}, {1.0e3, 8.0e3, 0.0}};
    w.shuttles[10] = {{1.5e11 + 7.0e6, 2.0e10, 0.0}, {3.0e4, -9.0e3, 0.0}};
    w.bary = {{1.0e9, -2.0e9, 0.0}, {10.0, 4.0, 0.0}};
    return w;
}

// Position error of the centre against the target, in metres. Positions are ~1.5e11 m, so the double ulp
// (3e-5 m) is the floor; the pan offset adds one more rounding.
constexpr double kCentreTol = 1e-4;

} // namespace

TEST_SUITE("app_camera_follow")
{
    TEST_CASE("following a body: the centre tracks the interpolated position")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setMetresPerPixel(1e3);
        cam.follow(TargetKind::Body, 1);
        CHECK(cam.following().kind == TargetKind::Body);
        CHECK(cam.following().id == 1);
        for (double t : {0.0, 0.016, 100.0, 86400.0, 3.0e7}) {
            const CameraUpdate u = cam.update(t, s);
            CHECK_FALSE(u.followLost);
            CHECK_FALSE(u.followTransferred);
            const Vec3 p = w.bodies[1].at(t);
            CHECK(std::fabs(cam.centreWorld().x - p.x) < kCentreTol);
            CHECK(std::fabs(cam.centreWorld().y - p.y) < kCentreTol);
            // and the target sits at the middle of the viewport
            double px = 0, py = 0;
            cam.worldToWindow(p, kVp, px, py);
            CHECK(std::fabs(px - 640.0) < 1e-6);
            CHECK(std::fabs(py - 360.0) < 1e-6);
            // the target is at the camera-relative origin: no jitter however fast it moves
            const Camera::Rel r = cam.toCameraRelative(p);
            CHECK(std::fabs(r.x) < 1e-3f);
            CHECK(std::fabs(r.y) < 1e-3f);
        }
    }

    TEST_CASE("a pan offset relative to the target is kept while it moves")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setMetresPerPixel(1e5);
        cam.follow(TargetKind::Body, 1);
        cam.update(0.0, s);
        cam.panByWindowPixels(-100.0, 40.0);        // drag left and down: the camera moves right (+x) and up (+y)
        CHECK(cam.panOffset().x == doctest::Approx(1e7));
        CHECK(cam.panOffset().y == doctest::Approx(4e6));
        cam.update(5000.0, s);
        const Vec3 p = w.bodies[1].at(5000.0);
        CHECK(std::fabs(cam.centreWorld().x - (p.x + 1e7)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (p.y + 4e6)) < kCentreTol);
        // zooming about a cursor away from the centre also goes into the offset and survives the next update
        double px = 0, py = 0;
        const Vec3 under = cam.windowToWorld(1000.0, 100.0, kVp);
        cam.zoomAtWindow(4.0, 1000.0, 100.0, kVp);
        cam.update(5000.0, s);
        cam.worldToWindow(under, kVp, px, py);
        CHECK(std::fabs(px - 1000.0) < 1e-4);
        CHECK(std::fabs(py - 100.0) < 1e-4);
    }

    TEST_CASE("follow() snaps to the target, followKeepingView() does not move the view")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setCentreWorld({1.0e10, 1.0e10, 0.0});
        const double t = 777.0;
        cam.followKeepingView(TargetKind::Body, 3, w.bodies[3].at(t));
        cam.update(t, s);
        CHECK(std::fabs(cam.centreWorld().x - 1.0e10) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - 1.0e10) < kCentreTol);
        // now it tracks body 3 with the offset it had
        cam.update(t + 1000.0, s);
        const Vec3 p0 = w.bodies[3].at(t), p1 = w.bodies[3].at(t + 1000.0);
        CHECK(std::fabs(cam.centreWorld().x - (1.0e10 + p1.x - p0.x)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (1.0e10 + p1.y - p0.y)) < kCentreTol);

        cam.follow(TargetKind::Body, 2);            // snap
        cam.update(t, s);
        CHECK(std::fabs(cam.centreWorld().x - w.bodies[2].at(t).x) < kCentreTol);
    }

    TEST_CASE("following a shuttle")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setMetresPerPixel(0.01);
        cam.follow(TargetKind::Shuttle, 10);
        for (double t : {0.0, 12.5, 600.0}) {
            cam.update(t, s);
            const Vec3 p = w.shuttles[10].at(t);
            CHECK(std::fabs(cam.centreWorld().x - p.x) < kCentreTol);
            CHECK(std::fabs(cam.centreWorld().y - p.y) < kCentreTol);
        }
        // a body with the same number is not the shuttle
        w.bodies[10] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        cam.update(700.0, s);
        CHECK(std::fabs(cam.centreWorld().x - w.shuttles[10].at(700.0).x) < kCentreTol);
    }

    TEST_CASE("target loss: a destroyed shuttle stops the follow and the centre stays put")
    {
        FakeWorld w = makeWorld();
        w.shuttles[10].dies = 100.0;
        const CameraSources s = w.sources();
        Camera cam;
        cam.follow(TargetKind::Shuttle, 10);
        cam.update(99.0, s);
        const Vec3 last = cam.centreWorld();
        const CameraUpdate u = cam.update(100.0, s);
        CHECK(u.followLost);
        CHECK_FALSE(u.followTransferred);
        CHECK_FALSE(cam.following().active());
        CHECK(cam.centreWorld().x == last.x);
        CHECK(cam.centreWorld().y == last.y);
        // and it stays there, frame after frame, and can be panned again
        const CameraUpdate u2 = cam.update(5000.0, s);
        CHECK_FALSE(u2.followLost);
        CHECK(cam.centreWorld().x == last.x);
        cam.panWorld(5.0, 0.0);
        cam.update(6000.0, s);
        CHECK(cam.centreWorld().x == last.x + 5.0);
    }

    TEST_CASE("target loss: a merged body hands the follow to its survivor without moving the view")
    {
        FakeWorld w = makeWorld();
        w.bodies[2].dies = 50.0;
        w.survivor[2] = 3;
        const CameraSources s = w.sources();
        Camera cam;
        cam.setMetresPerPixel(1e6);
        cam.follow(TargetKind::Body, 2);
        cam.update(0.0, s);
        cam.panByWindowPixels(30.0, 0.0);
        cam.update(49.0, s);
        const Vec3 before = cam.centreWorld();

        const CameraUpdate u = cam.update(50.0, s);
        CHECK(u.followTransferred);
        CHECK_FALSE(u.followLost);
        CHECK(cam.following().kind == TargetKind::Body);
        CHECK(cam.following().id == 3);
        // no jump at the merge instant (only the survivor's offset from the old centre changes the bookkeeping)
        CHECK(std::fabs(cam.centreWorld().x - before.x) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - before.y) < kCentreTol);
        // afterwards the camera moves with the survivor
        cam.update(1050.0, s);
        const Vec3 p0 = w.bodies[3].at(50.0), p1 = w.bodies[3].at(1050.0);
        CHECK(std::fabs(cam.centreWorld().x - (before.x + p1.x - p0.x)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (before.y + p1.y - p0.y)) < kCentreTol);
        // the transfer is reported once
        CHECK_FALSE(cam.update(1060.0, s).followTransferred);
    }

    TEST_CASE("target loss: survivors are followed transitively, and a cycle or dead end stops the follow")
    {
        FakeWorld w = makeWorld();
        w.bodies[2].dies = 10.0;
        w.bodies[3].dies = 20.0;
        w.survivor[2] = 3;
        w.survivor[3] = 1;                  // 2 merged into 3, then 3 into 1
        const CameraSources s = w.sources();
        Camera cam;
        cam.follow(TargetKind::Body, 2);
        cam.update(0.0, s);
        const CameraUpdate u = cam.update(30.0, s);     // both merges happened between two frames
        CHECK(u.followTransferred);
        CHECK(cam.following().id == 1);

        FakeWorld c = makeWorld();
        c.bodies[2].dies = 1.0;
        c.bodies[3].dies = 1.0;
        c.survivor[2] = 3;
        c.survivor[3] = 2;                  // corrupt cycle: must terminate and give up
        const CameraSources cs = c.sources();
        Camera cam2;
        cam2.follow(TargetKind::Body, 2);
        cam2.update(0.0, cs);
        CHECK(cam2.update(2.0, cs).followLost);

        FakeWorld d = makeWorld();
        d.bodies[2].dies = 1.0;
        d.survivor[2] = 99;                 // survivor id the world does not know
        const CameraSources ds = d.sources();
        Camera cam3;
        cam3.follow(TargetKind::Body, 2);
        cam3.update(0.0, ds);
        CHECK(cam3.update(2.0, ds).followLost);
        CHECK_FALSE(cam3.following().active());
    }

    TEST_CASE("following something that never existed, or with no sources, stops at once")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setCentreWorld({5.0, 6.0, 0.0});
        cam.follow(TargetKind::Body, 12345);
        CHECK(cam.update(0.0, s).followLost);
        CHECK(cam.centreWorld().x == 5.0);
        cam.follow(TargetKind::Body, 1);
        CHECK(cam.update(0.0, CameraSources{}).followLost);
        cam.follow(TargetKind::None, 1);                // ignored: not following
        CHECK_FALSE(cam.following().active());
        cam.follow(TargetKind::Body, 0);                // id 0 is never valid
        CHECK_FALSE(cam.following().active());
    }

    TEST_CASE("stopFollowing keeps the centre where it is")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.follow(TargetKind::Body, 1);
        cam.update(10.0, s);
        const Vec3 c = cam.centreWorld();
        cam.stopFollowing();
        CHECK_FALSE(cam.following().active());
        cam.update(100000.0, s);
        CHECK(cam.centreWorld().x == c.x);
        CHECK(cam.centreWorld().y == c.y);
    }

    TEST_CASE("frame origin: inertial is zero, barycentre and follow-body come from the callbacks")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        Vec3 o{9, 9, 9};
        CHECK(cam.frameMode() == FrameMode::Inertial);
        CHECK(cam.frameOrigin(123.0, s, o));
        CHECK(o.x == 0.0);
        CHECK(o.y == 0.0);
        CHECK(cam.frameVelocity(123.0, s, o));
        CHECK(o.x == 0.0);

        CHECK(cam.setFrame(FrameMode::Barycentre, 0, 0.0, s));
        CHECK(cam.frameOrigin(1000.0, s, o));
        CHECK(o.x == w.bary.at(1000.0).x);
        CHECK(o.y == w.bary.at(1000.0).y);
        CHECK(cam.frameVelocity(1000.0, s, o));
        CHECK(o.x == 10.0);
        CHECK(o.y == 4.0);

        CHECK(cam.setFrame(FrameMode::FollowBody, 2, 0.0, s));
        CHECK(cam.frameMode() == FrameMode::FollowBody);
        CHECK(cam.frameBody() == 2);
        CHECK(cam.frameOrigin(1000.0, s, o));
        CHECK(o.x == w.bodies[2].at(1000.0).x);
        CHECK(o.y == w.bodies[2].at(1000.0).y);
        CHECK(cam.frameVelocity(1000.0, s, o));
        CHECK(o.x == -5.0e3);
        CHECK(o.y == 2.0e3);

        // trail re-expression: a world point seen in the frame at time t is p - origin(t)
        const Vec3 p{1.0e11, 5.0e10, 0.0};
        const Vec3 f = cam.toFrame(p, 1000.0, s);
        CHECK(f.x == p.x - w.bodies[2].at(1000.0).x);
        CHECK(f.y == p.y - w.bodies[2].at(1000.0).y);

        // back to inertial
        CHECK(cam.setFrame(FrameMode::Inertial, 7, 0.0, s));
        CHECK(cam.frameBody() == 0);
        CHECK(cam.toFrame(p, 1000.0, s).x == p.x);
    }

    TEST_CASE("invalid frame requests are refused and change nothing")
    {
        FakeWorld w = makeWorld();
        w.bodies[3].dies = 5.0;
        const CameraSources s = w.sources();
        Camera cam;
        CHECK(cam.setFrame(FrameMode::Barycentre, 0, 0.0, s));
        CHECK_FALSE(cam.setFrame(FrameMode::FollowBody, 0, 0.0, s));      // follow-body needs a body
        CHECK_FALSE(cam.setFrame(FrameMode::FollowBody, 404, 0.0, s));    // unknown body
        CHECK_FALSE(cam.setFrame(FrameMode::FollowBody, 3, 6.0, s));      // dead at that time
        CHECK(cam.frameMode() == FrameMode::Barycentre);
        // a barycentre frame without a barycentre callback reports unavailable and acts as the origin
        CameraSources noBary = s;
        noBary.barycentre = nullptr;
        Vec3 o{1, 1, 1};
        CHECK_FALSE(cam.frameOrigin(0.0, noBary, o));
        CHECK(o.x == 0.0);
    }

    TEST_CASE("switching frame does not move the view; the centre then rides with the frame origin")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        cam.setCentreWorld({2.0e10, -3.0e10, 0.0});
        cam.update(0.0, s);

        // Inertial: the centre stays at its world position however long we wait
        cam.update(1.0e6, s);
        CHECK(cam.centreWorld().x == 2.0e10);

        cam.update(1.0e6, s);
        REQUIRE(cam.setFrame(FrameMode::Barycentre, 0, 1.0e6, s));
        CHECK(cam.centreWorld().x == 2.0e10);               // no jump at the switch
        cam.update(1.0e6, s);
        CHECK(std::fabs(cam.centreWorld().x - 2.0e10) < kCentreTol);
        // The barycentre drifts at (10, 4) m/s: after 1e5 s the camera has moved by (1e6, 4e5) m in world coordinates,
        // i.e. it stands still relative to the barycentre.
        cam.update(1.0e6 + 1.0e5, s);
        CHECK(std::fabs(cam.centreWorld().x - (2.0e10 + 1.0e6)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (-3.0e10 + 4.0e5)) < kCentreTol);

        // Panning in that frame moves the centre relative to the origin and it keeps riding afterwards
        cam.panWorld(1000.0, 0.0);
        cam.update(1.0e6 + 2.0e5, s);
        CHECK(std::fabs(cam.centreWorld().x - (2.0e10 + 2.0e6 + 1000.0)) < kCentreTol);

        // Follow-body frame with no follow target: same idea around body 3
        const double t = 1.0e6 + 2.0e5;
        REQUIRE(cam.setFrame(FrameMode::FollowBody, 3, t, s));
        const Vec3 c = cam.centreWorld();
        cam.update(t + 5000.0, s);
        CHECK(std::fabs(cam.centreWorld().x - (c.x + 1.0e3 * 5000.0)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (c.y + 8.0e3 * 5000.0)) < kCentreTol);
    }

    TEST_CASE("stopping a follow in a moving frame keeps the view, then the centre rides with the frame")
    {
        FakeWorld w = makeWorld();
        const CameraSources s = w.sources();
        Camera cam;
        REQUIRE(cam.setFrame(FrameMode::Barycentre, 0, 0.0, s));
        cam.follow(TargetKind::Body, 1);
        cam.update(100.0, s);
        const Vec3 c = cam.centreWorld();
        cam.stopFollowing();
        cam.update(100.0 + 1000.0, s);
        CHECK(std::fabs(cam.centreWorld().x - (c.x + 10.0 * 1000.0)) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - (c.y + 4.0 * 1000.0)) < kCentreTol);
    }

    TEST_CASE("the follow-body frame transfers to the survivor of a merge, or falls back to inertial")
    {
        FakeWorld w = makeWorld();
        w.bodies[2].dies = 50.0;
        w.survivor[2] = 3;
        w.bodies[3].dies = 80.0;                // 3 has no survivor
        const CameraSources s = w.sources();
        Camera cam;
        cam.setCentreWorld({1.0e10, 0.0, 0.0});
        REQUIRE(cam.setFrame(FrameMode::FollowBody, 2, 0.0, s));
        cam.update(10.0, s);
        const Vec3 c = cam.centreWorld();

        CameraUpdate u = cam.update(60.0, s);
        CHECK(u.frameTransferred);
        CHECK_FALSE(u.frameFellBack);
        CHECK(cam.frameMode() == FrameMode::FollowBody);
        CHECK(cam.frameBody() == 3);
        // the view did not jump at the transfer
        CHECK(std::fabs(cam.centreWorld().x - c.x) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - c.y) < kCentreTol);

        const Vec3 c2 = cam.centreWorld();
        u = cam.update(90.0, s);
        CHECK(u.frameFellBack);
        CHECK(cam.frameMode() == FrameMode::Inertial);
        CHECK(cam.frameBody() == 0);
        CHECK(std::fabs(cam.centreWorld().x - c2.x) < kCentreTol);
        CHECK(std::fabs(cam.centreWorld().y - c2.y) < kCentreTol);
        // and stays put from now on
        cam.update(1.0e5, s);
        CHECK(std::fabs(cam.centreWorld().x - c2.x) < kCentreTol);
    }
}
