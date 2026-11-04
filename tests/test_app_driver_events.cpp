#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "core/atmosphere.h"
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// Events and removals in the SimDriver (plan 4.3, 4.7, 8): the physics runs up to one body step ahead of the display, so
// an event must reach the effects only when the display time reaches it, and a shuttle or body that is about to be
// destroyed or absorbed must still be drawn until then.

namespace {

DriverOptions unlimited()
{
    DriverOptions o;
    o.physicsBudget_s = 0.0;
    return o;
}

} // namespace

TEST_SUITE("app driver events")
{
    TEST_CASE("a shuttle destroyed in the step ahead is drawn until its contact time; the event is due only then")
    {
        scene_test::Lucerna L;
        const std::size_t domus = L.storeIndex("Domus");
        ResolvedScene res = std::move(L.res);
        const BodyId domusId = res.bodies.ids[domus];
        const double muD = res.bodies.mu[domus];
        const double RD = res.bodies.radius[domus];
        Vec3 pos, vel;
        sim_test::circularOrbit(muD, RD + 1.5e5, 1.0, sim_test::bodyPos(res.bodies, domus), sim_test::bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(sim_test::resolvedShuttle(900, pos, vel, "Retrograde"));

        SimulationOptions opt;
        opt.ias15.dtMax = 120.0;   // body steps of up to 120 s, so the physics is well ahead of 1 s frames
        SimDriver d(res, SceneSettings{}, opt, unlimited());
        d.warp().set(8.0);   // with dt = 1/8 s: 1 s of simulated time per frame

        std::vector<Event> due;
        bool sawAheadFrame = false, sawDying = false;
        double contactT = 0.0;
        bool sawGone = false;
        for (int frame = 0; frame < 8000 && !sawGone; ++frame) {
            const FrameInfo& f = d.advance(0.125);
            // events are due only once the display has reached them, and each exactly once
            for (const Event& e : d.dueEvents()) {
                CHECK(e.t <= f.target);
                due.push_back(e);
            }
            for (const Event& e : d.drainedThisFrame()) {
                if (e.type != EventType::ShuttleDestroyed) continue;
                contactT = e.t;
                if (e.t > f.target) {
                    sawAheadFrame = true;
                    // physics is ahead: the event is known but not due, and the shuttle is still on screen
                    for (const Event& x : d.dueEvents()) CHECK(x.type != EventType::ShuttleDestroyed);
                }
            }
            const DisplayShuttle* sh = findShuttle(d.state(), 900);
            if (sh && sh->dying) sawDying = true;
            if (contactT > 0.0 && f.target >= contactT) {
                CHECK(sh == nullptr);   // gone from the display as soon as the display reaches the contact
                sawGone = true;
            } else if (contactT > 0.0) {
                REQUIRE(sh != nullptr);   // known to die ahead, still drawn
                CHECK(sh->dying);
            }
        }
        REQUIRE(contactT > 0.0);
        REQUIRE(sawGone);
        CHECK(sawAheadFrame);
        CHECK(sawDying);
        CHECK(d.pendingEventCount() == 0);

        // one AtmosphereEntry then one ShuttleDestroyed, each once, in time order, both about Domus
        std::size_t entries = 0, destroyed = 0;
        for (std::size_t i = 0; i < due.size(); ++i) {
            if (i > 0) CHECK(due[i - 1].t <= due[i].t);
            if (due[i].type == EventType::AtmosphereEntry) {
                ++entries;
                CHECK(due[i].other == domusId);
                CHECK(destroyed == 0);
            }
            if (due[i].type == EventType::ShuttleDestroyed) {
                ++destroyed;
                CHECK(due[i].id == 900);
                CHECK(due[i].other == domusId);
            }
        }
        CHECK(entries == 1);
        CHECK(destroyed == 1);

        // Just before the contact the drawn shuttle is at Domus's surface: the last Hermite sample is the contact point
        const DisplayState before = d.stateAt(contactT - 1.0e-6);
        const DisplayShuttle* sh = findShuttle(before, 900);
        const DisplayBody* dom = findBody(before, domusId);
        REQUIRE(sh != nullptr);
        REQUIRE(dom != nullptr);
        CHECK(sh->dying);
        const double altitude = norm(sh->r - dom->r) - RD;
        MESSAGE("altitude just before the contact: " << altitude << " m");
        // The shuttle touches Domus when its centre is one shuttle radius (2 m) above the surface. Measured: 8e-5 m off, which
        // is what 1e-6 s at the impact speed (a few km/s, so a few mm at most) and the Hermite fit leave.
        CHECK(std::fabs(altitude - res.shuttles[0].desc.radius) < 0.01);
        // At the contact time itself it is no longer drawn
        CHECK(findShuttle(d.stateAt(contactT), 900) == nullptr);
    }

    TEST_CASE("a body absorbed in the step ahead is drawn until the merge time; the survivor keeps its look until then")
    {
        // A heavy and a light body falling towards each other along x
        const double muA = 1.0e14, muB = 1.0e10, rA = 1.0e5, rB = 1.0e5;
        const ResolvedScene res = sim_test::sceneOf({sim_test::bodyDesc(muA, rA, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}),
                                                     sim_test::bodyDesc(muB, rB, {4.0e6, 0.0, 0.0}, {-2000.0, 0.0, 0.0})});
        const BodyId idA = res.bodyIds[0], idB = res.bodyIds[1];
        SimDriver d(res, SceneSettings{}, SimulationOptions{}, unlimited());

        // Walk to the body step that ends in the contact: one body step per call, the display just after its start.
        double mergeT = 0.0;
        for (int i = 0; i < 100000 && mergeT == 0.0; ++i) {
            d.advanceTo(justAfter(d.simTime()));
            for (const Event& e : d.drainedThisFrame())
                if (e.type == EventType::Merge) {
                    mergeT = e.t;
                    CHECK(e.id == idA);   // the heavier one survives
                    CHECK(e.other == idB);
                }
        }
        REQUIRE(mergeT > 0.0);
        // The contact: the radii touch (2e5 m apart); falling from 4e6 m with up to ~4.6 km/s takes about 650 s
        CHECK(mergeT > 500.0);
        CHECK(mergeT < 800.0);
        // The physics has merged them; the display is still before that time
        REQUIRE(d.targetTime() < mergeT);
        REQUIRE(d.sim().bodies().size() == 1);
        CHECK(d.dueEvents().empty());

        // Before the merge time the pair is still drawn, as at the start of the step: sampled at the display time and
        // inside the step the contact ends (the merge step is short, so the display crosses it in one frame)
        const double t0 = d.sim().bodyStepStart();
        REQUIRE(t0 < mergeT);
        const DisplayState atStart = d.stateAt(t0);
        DisplayState previous = atStart;
        const int samples = 20;
        for (int k = 0; k <= samples; ++k) {
            const double t = k == samples ? mergeT - 1.0e-9 : t0 + (mergeT - t0) * k / samples;
            const DisplayState s = d.stateAt(t);
            REQUIRE(s.bodies.size() == 2);
            const DisplayBody* a = findBody(s, idA);
            REQUIRE(a != nullptr);
            CHECK(a->radius == rA);   // the survivor has not grown yet
            CHECK(a->mu == muA);
            const DisplayBody* ghost = nullptr;
            for (const DisplayBody& b : s.bodies)
                if (b.absorbed) ghost = &b;
            REQUIRE(ghost != nullptr);
            CHECK(ghost->id == idB);
            CHECK(ghost->interpolated);
            CHECK(ghost->radius == rB);
            CHECK(ghost->mu == muB);
            // continuous: it moves by what its speed allows (up to ~4.6 km/s)
            const DisplayBody* pb = nullptr;
            for (const DisplayBody& b : previous.bodies)
                if (b.absorbed) pb = &b;
            if (!pb) pb = findBody(previous, idB);
            REQUIRE(pb != nullptr);
            CHECK(norm(ghost->r - pb->r) <= 4800.0 * (mergeT - t0) / samples + 1.0e-3);
            previous = s;
        }
        // The two touch at the merge time: centres one radius sum apart (the clipped step ends at the contact)
        {
            const DisplayState s = d.stateAt(mergeT - 1.0e-9);
            const DisplayBody* a = findBody(s, idA);
            const DisplayBody* ghost = nullptr;
            for (const DisplayBody& b : s.bodies)
                if (b.absorbed) ghost = &b;
            REQUIRE(ghost != nullptr);
            CHECK(norm(ghost->r - a->r) == doctest::Approx(rA + rB).epsilon(1e-6));
        }

        // Frames from here on: the merge becomes due exactly once, then there is one body
        bool sawMergeDue = false;
        const double dt = 0.0625;   // warp 1
        for (int frame = 0; frame < 100 && !sawMergeDue; ++frame) {
            const FrameInfo& f = d.advance(dt);
            const DisplayState& s = d.state();
            std::size_t mergesDue = 0;
            for (const Event& e : d.dueEvents()) mergesDue += e.type == EventType::Merge ? 1 : 0;
            if (f.target < mergeT) {
                CHECK(mergesDue == 0);
                CHECK(s.bodies.size() == 2);
            } else {
                sawMergeDue = true;
                CHECK(mergesDue == 1);
                REQUIRE(s.bodies.size() == 1);
                CHECK_FALSE(s.bodies[0].absorbed);
                CHECK(s.bodies[0].id == idA);
                CHECK(s.bodies[0].mu == muA + muB);
                CHECK(s.bodies[0].radius > rA);
            }
        }
        CHECK(sawMergeDue);
        // afterwards the one body is just drawn, and the event is not reported again
        for (int i = 0; i < 10; ++i) {
            d.advance(dt);
            CHECK(d.state().bodies.size() == 1);
            for (const Event& e : d.dueEvents()) CHECK(e.type != EventType::Merge);
        }
    }
}
