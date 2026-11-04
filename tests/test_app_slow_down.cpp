#include <doctest.h>
#include <cmath>
#include <vector>
#include "app/slow_down.h"
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// Warp step-down (plan section 8 items 4-7): predictions from the display state, one ladder level per frame, 1x about 3 real
// seconds before the impact, nothing for unwatched shuttles, nothing in headless runs, physics untouched.

namespace {

DriverOptions unlimited()
{
    DriverOptions o;
    o.physicsBudget_s = 0.0;
    return o;
}

constexpr double kFrame = 1.0 / 64.0;   // real seconds per frame (exact binary fraction)
constexpr std::uint64_t kFaller = 900;

// Lucerna plus one shuttle falling straight onto Domus from rest (relative to Domus), 2e7 m above the surface: about 7400 s of
// free fall to the top of the atmosphere, a clean test of the step-down.
ResolvedScene lucernaWithFaller()
{
    scene_test::Lucerna L;
    const std::size_t domus = L.storeIndex("Domus");
    ResolvedScene res = std::move(L.res);
    const double R = res.bodies.radius[domus];
    const Vec3 pD = sim_test::bodyPos(res.bodies, domus), vD = sim_test::bodyVel(res.bodies, domus);
    res.shuttles.push_back(sim_test::resolvedShuttle(kFaller, pD + Vec3{R + 2.0e7, 0.0, 0.0}, vD, "Coast"));
    return res;
}

struct Run {
    std::vector<double> warps;        // distinct warp values seen, in order
    double realAtOne = -1.0;          // real time (s) at which the warp first became 1
    double realAtEntry = -1.0;        // real time at which the AtmosphereEntry became due
    double realAtDestroyed = -1.0;
    double realNow = 0.0;
    bool sawEntry = false;
};

// Frames of kFrame real seconds until the shuttle enters the air (or maxFrames). `slow` is updated after each advance, like the app.
Run runFall(SimDriver& d, SlowDown& slow, const std::vector<std::uint64_t>& watched, int maxFrames = 4000)
{
    Run run;
    run.warps.push_back(d.warp().value());
    for (int f = 0; f < maxFrames && !run.sawEntry; ++f) {
        d.advance(kFrame);
        run.realNow += kFrame;
        for (const Event& e : d.dueEvents()) {
            if (e.type == EventType::AtmosphereEntry && e.id == kFaller) {
                run.sawEntry = true;
                run.realAtEntry = run.realNow;
            }
        }
        if (run.sawEntry) break;
        slow.update(d, watched);
        if (d.warp().value() != run.warps.back()) {
            run.warps.push_back(d.warp().value());
            if (d.warp().value() == 1.0 && run.realAtOne < 0.0) run.realAtOne = run.realNow;
        }
    }
    return run;
}

} // namespace

TEST_SUITE("app slow down")
{
    TEST_CASE("nextLowerWarp follows the ladder")
    {
        CHECK(nextLowerWarp(1.0e6) == 1.0e5);
        CHECK(nextLowerWarp(100.0) == 10.0);
        CHECK(nextLowerWarp(250.0) == 100.0);
        CHECK(nextLowerWarp(10.0) == 1.0);
        CHECK(nextLowerWarp(1.0) == 1.0);
        CHECK(nextLowerWarp(0.5) == 0.5);
    }

    TEST_CASE("a falling shuttle is stepped down level by level and is at 1x about 3 real seconds before the air")
    {
        SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1.0e5);
        SlowDown slow;
        const Run run = runFall(d, slow, {kFaller});
        REQUIRE(run.sawEntry);

        // every rung of the ladder, once, in order
        REQUIRE(run.warps.size() == 6);
        CHECK(run.warps[0] == 1.0e5);
        CHECK(run.warps[1] == 1.0e4);
        CHECK(run.warps[2] == 1.0e3);
        CHECK(run.warps[3] == 100.0);
        CHECK(run.warps[4] == 10.0);
        CHECK(run.warps[5] == 1.0);

        // at 1x with about 3 s to go (prediction is the osculating two-body conic; the real flight differs a little)
        REQUIRE(run.realAtOne > 0.0);
        const double left = run.realAtEntry - run.realAtOne;
        CHECK(left > 2.0);
        CHECK(left < 4.5);

        // the Resume warp button returns to the pre-slow-down warp
        CHECK(d.warp().canResume());
        CHECK(d.warp().resumeValue() == 1.0e5);
        CHECK(d.warp().resume());
        CHECK(d.warp().value() == 1.0e5);
        CHECK_FALSE(d.warp().canResume());
    }

    TEST_CASE("one ladder level per update, and the report names the cause")
    {
        SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1.0e6);
        d.advance(kFrame);
        SlowDown slow;
        const SlowDownReport r1 = slow.update(d, {kFaller});
        CHECK(r1.cause == SlowDownReport::Cause::Shuttle);
        CHECK(r1.subject == kFaller);
        CHECK(r1.steppedDown);
        CHECK(r1.warpBefore == 1.0e6);
        CHECK(r1.warpAfter == 1.0e5);   // exactly one level, however close the impact is
        CHECK(r1.timeToImpact > 3.0e4);
        CHECK(r1.timeToImpact < 4.0e4);
        // same frame state: 1e5 -> 1e4 needs the impact inside 3e4 s, which it is not yet (36000 s)
        const SlowDownReport r2 = slow.update(d, {kFaller});
        CHECK_FALSE(r2.steppedDown);
        CHECK(r2.warpAfter == 1.0e5);
    }

    TEST_CASE("an unwatched shuttle does not slow the warp unless the any-shuttle switch is on")
    {
        {
            SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
            d.warp().set(1.0e5);
            SlowDown slow;   // anyShuttle off by default
            const Run run = runFall(d, slow, {});
            REQUIRE(run.sawEntry);
            CHECK(run.warps.size() == 1);
            CHECK(d.warp().value() == 1.0e5);
            CHECK_FALSE(d.warp().canResume());
        }
        {
            SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
            d.warp().set(1.0e5);
            SlowDown slow;
            slow.options.anyShuttle = true;
            const Run run = runFall(d, slow, {});
            REQUIRE(run.sawEntry);
            CHECK(run.warps.size() == 6);
            CHECK(d.warp().value() == 1.0);
        }
        {
            // watching some other shuttle id does not count either
            SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
            d.warp().set(1.0e5);
            SlowDown slow;
            const Run run = runFall(d, slow, {12345});
            CHECK(run.warps.size() == 1);
        }
    }

    TEST_CASE("headless runs never slow down")
    {
        SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1.0e5);
        SlowDown slow;
        slow.options.enabled = false;
        slow.options.anyShuttle = true;
        const Run run = runFall(d, slow, {kFaller});
        REQUIRE(run.sawEntry);
        CHECK(run.warps.size() == 1);
        CHECK_FALSE(d.warp().canResume());
        // and the driver on its own never touches the warp
        SimDriver e(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
        e.warp().set(1.0e5);
        for (int i = 0; i < 100; ++i) e.advance(kFrame);
        CHECK(e.warp().value() == 1.0e5);
    }

    TEST_CASE("no slow-down while paused or at 1x, and none when nothing approaches")
    {
        SimDriver d(lucernaWithFaller(), SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1.0e5);
        d.advance(kFrame);
        SlowDown slow;
        d.setPaused(true);
        CHECK_FALSE(slow.update(d, {kFaller}).steppedDown);
        d.setPaused(false);
        d.warp().set(1.0);
        CHECK_FALSE(slow.update(d, {kFaller}).steppedDown);

        // Lucerna with a shuttle in a circular orbit and no collision course
        SimDriver q(driver_test::lucernaWithShuttles(1), SceneSettings{}, SimulationOptions{}, unlimited());
        q.warp().set(1.0e5);
        q.advance(kFrame);
        const SlowDownReport r = slow.update(q, {900});
        CHECK(r.cause == SlowDownReport::Cause::None);
        CHECK_FALSE(r.steppedDown);
        CHECK(q.warp().value() == 1.0e5);
    }

    TEST_CASE("a body pair on a collision course steps the warp down, 1x about 3 s before the merge")
    {
        // A heavy body at rest and a light one falling straight onto it from 1e8 m: about 1.1e5 s of fall
        ResolvedScene res = sim_test::sceneOf({sim_test::bodyDesc(1.0e14, 1.0e6, {0, 0, 0}, {0, 0, 0}),
                                               sim_test::bodyDesc(1.0e9, 1.0e5, {1.0e8, 0, 0}, {0, 0, 0})});
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1.0e5);
        SlowDown slow;
        std::vector<double> warps = {1.0e5};
        double real = 0.0, realAtOne = -1.0, realAtMerge = -1.0;
        for (int f = 0; f < 6000 && realAtMerge < 0.0; ++f) {
            d.advance(kFrame);
            real += kFrame;
            for (const Event& e : d.dueEvents())
                if (e.type == EventType::Merge) realAtMerge = real;
            if (realAtMerge >= 0.0) break;
            const SlowDownReport r = slow.update(d, {});   // no shuttle watched: the pair alone triggers it
            if (r.steppedDown) {
                CHECK(r.cause == SlowDownReport::Cause::BodyPair);
                warps.push_back(d.warp().value());
                if (d.warp().value() == 1.0) realAtOne = real;
            }
        }
        REQUIRE(realAtMerge > 0.0);
        REQUIRE(warps.size() == 6);
        REQUIRE(realAtOne > 0.0);
        const double left = realAtMerge - realAtOne;
        CHECK(left > 2.0);
        CHECK(left < 4.5);
        CHECK(d.warp().canResume());
        CHECK(d.warp().resumeValue() == 1.0e5);
    }

    TEST_CASE("the slow-down does not change the physics")
    {
        // the same final display time reached with and without slow-downs gives the same bits (physics is warp independent)
        const ResolvedScene res = lucernaWithFaller();
        SimDriver a(res, SceneSettings{}, SimulationOptions{}, unlimited());
        a.warp().set(1.0e5);
        SlowDown slow;
        for (int i = 0; i < 300; ++i) {
            a.advance(kFrame);
            slow.update(a, {kFaller});
        }
        SimDriver b(res, SceneSettings{}, SimulationOptions{}, unlimited());
        b.advanceTo(a.targetTime());
        CHECK(sameBits(a.state(), b.state()));
    }
}
