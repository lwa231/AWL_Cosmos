#include <doctest.h>
#include <cmath>
#include <utility>
#include <vector>
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// Frame loop of the SimDriver: warp-schedule independence of the render state (plan 4.4, 4.6), pause, clamp, edits,
// and the per-frame physics cap (plan 8 "never changes the trajectory").

namespace {

DriverOptions unlimited()
{
    DriverOptions o;
    o.physicsBudget_s = 0.0;   // no wall-time cap: the result must not depend on how fast the machine is
    return o;
}

struct Frame {
    double dt;     // real seconds (exact binary fraction, below the 0.25 s clamp)
    double warp;   // integer warp: dt * warp is exact, so every schedule sums to the same t_target exactly
};

// Run the frames; returns the driver's final display time.
void runFrames(SimDriver& d, const std::vector<std::pair<Frame, int>>& plan)
{
    for (const auto& step : plan)
        for (int i = 0; i < step.second; ++i) {
            d.warp().set(step.first.warp);
            d.advance(step.first.dt);
        }
}

} // namespace

TEST_SUITE("app driver schedule")
{
    TEST_CASE("the same t_target gives the same state for any frame and warp schedule, thread count and path")
    {
        const double T = 200000.0;   // about 150 body steps, shuttles thrusting
        const ResolvedScene res = lucernaWithShuttles(3);

        // A: steady 125 s per frame
        SimDriver a(res, SceneSettings{}, SimulationOptions{}, unlimited());
        runFrames(a, {{{0.125, 1000.0}, 1600}});
        // B: one huge frame
        SimDriver b(res, SceneSettings{}, SimulationOptions{}, unlimited());
        runFrames(b, {{{0.25, 800000.0}, 1}});
        // C: a mixed schedule across the whole ladder, on 4 threads; remember the state half way
        SimulationOptions four;
        four.threads = 4;
        SimDriver c(res, SceneSettings{}, four, unlimited());
        runFrames(c, {{{0.125, 10000.0}, 40}, {{0.25, 100000.0}, 2}});   // 50000 + 50000
        const double halfC = c.targetTime();
        const DisplayState halfStateC = c.state();
        runFrames(c, {{{0.125, 100.0}, 4000}, {{0.0625, 800000.0}, 1}});   // 50000 + 50000
        // D: single steps of 1 s (warp 8, dt 1/8) for the first 100 s, then big frames; 1 thread
        SimDriver e(res, SceneSettings{}, SimulationOptions{}, unlimited());
        runFrames(e, {{{0.125, 8.0}, 100}, {{0.25, 79600.0}, 10}});   // 100 + 199000 = 199100
        e.advanceTo(T);                                                                         // ... and the rest in one jump

        // every schedule landed on the same display time exactly
        CHECK(a.targetTime() == T);
        CHECK(b.targetTime() == T);
        CHECK(c.targetTime() == T);
        CHECK(e.targetTime() == T);

        CHECK(sameBits(a.state(), b.state()));
        CHECK(sameBits(a.state(), c.state()));
        CHECK(sameBits(a.state(), e.state()));
        // the physics itself was the same, so the end of the latest body step is the same too
        CHECK(sim_test::sameBits(a.simTime(), b.simTime()));
        CHECK(sim_test::sameBits(a.simTime(), c.simTime()));
        CHECK(sim_test::sameBits(a.simTime(), e.simTime()));
        CHECK(sim_test::sameBits(a.sim().bodies().state, c.sim().bodies().state));
        // and the state can be read again at T without changing it
        CHECK(sameBits(a.stateAt(T), a.state()));
        // the shuttles really moved (the test would be vacuous on a static scene)
        REQUIRE(a.state().shuttles.size() == 3);
        CHECK(a.state().shuttles[0].interpolated);
        CHECK(a.simTime() >= T);

        // half way: the mixed schedule C is at 100000 s; the steady schedule A is there after 800 of its frames
        REQUIRE(halfC == 100000.0);
        SimDriver a2(res, SceneSettings{}, SimulationOptions{}, unlimited());
        runFrames(a2, {{{0.125, 1000.0}, 800}});
        CHECK(a2.targetTime() == halfC);
        CHECK(sameBits(a2.state(), halfStateC));
    }

    TEST_CASE("a frame is clamped to 0.25 s and a pause freezes the state")
    {
        const ResolvedScene res = lucernaWithShuttles(2);
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(100.0);
        const FrameInfo f = d.advance(5.0);   // a 5 s stall counts as 0.25 s
        CHECK(f.frameDt == 0.25);
        CHECK(f.warp == 100.0);
        CHECK(f.simDelta == 25.0);
        CHECK(d.targetTime() == 25.0);
        CHECK(d.state().t == 25.0);

        const DisplayState before = d.state();
        d.setPaused(true);
        CHECK(d.paused());
        for (int i = 0; i < 5; ++i) {
            const FrameInfo& p = d.advance(0.01);
            CHECK(p.simDelta == 0.0);
            CHECK(p.bodySteps == 0);
            CHECK_FALSE(p.physicsBehind);
        }
        CHECK(d.targetTime() == 25.0);
        CHECK(sameBits(d.state(), before));
        d.setPaused(false);
        d.advance(0.125);
        CHECK(d.targetTime() == 25.0 + 12.5);
    }

    TEST_CASE("a warp change applies from the next frame on")
    {
        const ResolvedScene res = lucernaWithShuttles(1);
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, unlimited());
        d.advance(0.125);
        CHECK(d.frameInfo().warp == 1.0);
        d.warp().set(10.0);
        CHECK(d.targetTime() == 0.125);   // nothing already counted moves
        d.advance(0.125);
        CHECK(d.frameInfo().warp == 10.0);
        CHECK(d.targetTime() == 0.125 + 1.25);
        d.warp().set(1000.0);
        d.advance(0.125);
        CHECK(d.frameInfo().simDelta == 125.0);
        CHECK(d.targetTime() == 0.125 + 1.25 + 125.0);
        // physics ran ahead of the display, never behind it
        CHECK(d.simTime() >= d.targetTime());
    }

    TEST_CASE("a shuttle added while paused is drawn where it starts, without interpolation")
    {
        const ResolvedScene res = lucernaWithShuttles(1);
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, unlimited());
        d.warp().set(1000.0);
        for (int i = 0; i < 20; ++i) d.advance(0.125);
        d.setPaused(true);
        REQUIRE(d.state().shuttles.size() == 1);
        CHECK(d.state().shuttles[0].interpolated);

        ShuttleDesc desc;
        desc.position = {5.0e11, 1.0e9, 0.0};
        desc.velocity = {0.0, 100.0, 0.0};
        const ShuttleId id = d.sim().addShuttle(desc);
        REQUIRE(id != 0);
        d.advance(0.0);   // refresh the state after the edit
        REQUIRE(d.state().shuttles.size() == 2);
        const DisplayShuttle* s = findShuttle(d.state(), id);
        REQUIRE(s != nullptr);
        CHECK_FALSE(s->interpolated);
        CHECK(sameBits(s->r, desc.position));
        CHECK(sameBits(s->v, desc.velocity));

        d.setPaused(false);
        for (int i = 0; i < 40; ++i) d.advance(0.125);
        const DisplayShuttle* later = findShuttle(d.state(), id);
        REQUIRE(later != nullptr);
        CHECK(later->interpolated);
        CHECK(norm(later->r - desc.position) > 1.0e5);   // 5000 s at 100 m/s and more
    }

    TEST_CASE("when the physics cannot keep up, the display time waits and the trajectory is untouched")
    {
        const ResolvedScene res = lucernaWithShuttles(3);
        DriverOptions tiny;
        tiny.physicsBudget_s = 1.0e-9;   // below the cost of one body step: one step per frame at most when behind
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, tiny);
        d.warp().set(1.0e6);

        int behindFrames = 0;
        double previousTarget = d.targetTime();
        for (int i = 0; i < 40; ++i) {
            const FrameInfo f = d.advance(0.25);   // wants 250000 s = about 190 body steps
            if (f.physicsBehind) {
                ++behindFrames;
                CHECK(f.bodySteps >= 1);
                CHECK(f.lagSimSeconds > 0.0);
                // the display time waits at the physics time: it never shows a state the physics has not computed
                CHECK(f.target == f.simTime);
                CHECK(f.simDelta < 0.25 * 1.0e6);
                CHECK(f.physicsSeconds >= tiny.physicsBudget_s);
            }
            CHECK(f.target >= previousTarget);   // time never runs backwards
            CHECK(d.state().t == f.target);
            previousTarget = f.target;
        }
        CHECK(behindFrames == 40);
        CHECK(d.physicsBehind());
        CHECK(d.targetTime() < 40 * 0.25e6);

        // The trajectory is the one an uncapped run takes: same bits at the same physics time
        Simulation reference(res, SceneSettings{}, [] {
            SimulationOptions o;
            o.recordShuttlePaths = true;
            return o;
        }());
        REQUIRE(reference.advanceTo(d.simTime()));
        CHECK(sim_test::sameBits(reference.time(), d.simTime()));
        CHECK(sim_test::sameBits(reference.bodies().state, d.sim().bodies().state));
        CHECK(sim_test::sameBits(reference.shuttles().state, d.sim().shuttles().state));

        // Back at 1x the display catches up at once: no longer behind, display time advances by exactly warp * dt
        d.warp().set(1.0);
        const double t = d.targetTime();
        const FrameInfo& f = d.advance(0.125);
        CHECK_FALSE(f.physicsBehind);
        CHECK(f.simDelta == 0.125);
        CHECK(d.targetTime() == t + 0.125);
    }

    TEST_CASE("with no cap the physics always reaches the target; a normal budget is not exceeded by a gentle warp")
    {
        const ResolvedScene res = lucernaWithShuttles(3);
        SimDriver capped(res, SceneSettings{});   // default 25 ms budget
        capped.warp().set(1000.0);
        for (int i = 0; i < 60; ++i) {
            const FrameInfo& f = capped.advance(1.0 / 144.0);
            CHECK_FALSE(f.physicsBehind);
            CHECK(f.simTime >= f.target);
            CHECK(f.simDelta == doctest::Approx(1000.0 / 144.0).epsilon(1e-12));
        }

        SimDriver uncapped(res, SceneSettings{}, SimulationOptions{}, unlimited());
        uncapped.warp().set(1.0e6);
        const FrameInfo& f = uncapped.advance(0.25);
        CHECK_FALSE(f.physicsBehind);
        CHECK(f.simTime >= f.target);
        CHECK(f.target == 250000.0);
    }
}
