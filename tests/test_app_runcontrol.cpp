#include <doctest.h>
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

// Run control (plan Q17, Q19, 10.3): the Stopped / Running / Paused state machine.
TEST_SUITE("app runcontrol")
{
    TEST_CASE("a new RunControl is stopped with the empty sandbox")
    {
        RunControl rc(options());
        CHECK(rc.state() == RunState::Stopped);
        CHECK(rc.canEdit());
        CHECK_FALSE(rc.running());
        CHECK(rc.editor().scene().bodies.empty());
        CHECK(rc.editor().scene().name == "Empty sandbox");
        CHECK(rc.driver().state().bodies.empty());
        CHECK(rc.snapshot() == nullptr);
        CHECK_FALSE(rc.dirty());
        CHECK(rc.generation() > 0);
        CHECK(std::string(runStateName(rc.state())) == "Stopped");
    }

    TEST_CASE("Start refuses an empty scene")
    {
        RunControl rc(options());
        const EditResult r = rc.start();
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("Add at least one body") != std::string::npos);
        CHECK(rc.state() == RunState::Stopped);
        CHECK(rc.lastError() == r.error);
    }

    TEST_CASE("the transitions")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(2), err));
        CHECK(rc.state() == RunState::Stopped);

        CHECK_FALSE(rc.pause().ok);   // nothing to pause
        CHECK(rc.stop().ok);          // already stopped: harmless
        CHECK(rc.state() == RunState::Stopped);

        REQUIRE(rc.start().ok);
        CHECK(rc.state() == RunState::Running);
        CHECK(rc.running());
        CHECK_FALSE(rc.canEdit());
        CHECK(rc.snapshot() != nullptr);
        CHECK(rc.start().ok);   // already running: nothing happens
        CHECK(rc.state() == RunState::Running);

        REQUIRE(rc.pause().ok);
        CHECK(rc.state() == RunState::Paused);
        CHECK(rc.canEdit());
        CHECK_FALSE(rc.pause().ok);   // twice

        REQUIRE(rc.start().ok);   // resume
        CHECK(rc.state() == RunState::Running);
        REQUIRE(rc.stop().ok);
        CHECK(rc.state() == RunState::Stopped);

        REQUIRE(rc.start().ok);
        REQUIRE(rc.pause().ok);
        REQUIRE(rc.stop().ok);   // stop from paused
        CHECK(rc.state() == RunState::Stopped);
        CHECK(rc.canEdit());
    }

    TEST_CASE("Start takes the snapshot; Stop restores it bit for bit")
    {
        RunControl rc(options());
        std::string err;
        const Scene scene = lucernaWithShuttles(3);
        REQUIRE(rc.loadScene(scene, err));
        rc.driver().warp().set(1.0e4);

        const DisplayState before = rc.driver().state();
        const Scene sceneBefore = rc.editor().scene();
        CHECK(sameScene(sceneBefore, scene));
        const double epoch = scene.epoch_s;

        REQUIRE(rc.start().ok);
        REQUIRE(rc.snapshot() != nullptr);
        CHECK(sameScene(*rc.snapshot(), scene));
        runFrames(rc, 40);
        CHECK(rc.driver().targetTime() > epoch + 1.0e4);   // it ran
        CHECK_FALSE(driver_test::sameBits(rc.driver().state(), before));

        REQUIRE(rc.stop().ok);
        CHECK(rc.state() == RunState::Stopped);
        CHECK(driver_test::sameBits(rc.driver().state(), before));
        CHECK(rc.driver().targetTime() == epoch);
        CHECK(rc.driver().simTime() == epoch);
        CHECK(sameScene(rc.editor().scene(), scene));
        CHECK(rc.driver().warp().value() == 1.0e4);   // the warp setting survives
        CHECK_FALSE(rc.dirty());
        CHECK(rc.editor().recentre() == Recentre::Auto);
        CHECK_FALSE(rc.editor().locked());
    }

    TEST_CASE("two runs from the same start are identical")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(3), err));
        rc.driver().warp().set(1.0e4);

        REQUIRE(rc.start().ok);
        runFrames(rc, 30);
        const DisplayState first = rc.driver().state();
        REQUIRE(rc.stop().ok);

        REQUIRE(rc.start().ok);
        runFrames(rc, 30);
        CHECK(driver_test::sameBits(rc.driver().state(), first));
    }

    TEST_CASE("pause and resume without an edit do not disturb the run at all")
    {
        RunControl straight(options()), paused(options());
        std::string err;
        REQUIRE(straight.loadScene(lucernaWithShuttles(3), err));
        REQUIRE(paused.loadScene(lucernaWithShuttles(3), err));
        straight.driver().warp().set(1.0e4);
        paused.driver().warp().set(1.0e4);

        REQUIRE(straight.start().ok);
        runFrames(straight, 40);

        REQUIRE(paused.start().ok);
        runFrames(paused, 15);
        REQUIRE(paused.pause().ok);
        const std::uint64_t generation = paused.generation();
        const SimDriver* driverPtr = &paused.driver();
        const DisplayState frozen = paused.driver().state();
        runFrames(paused, 10);   // frames while paused change nothing
        CHECK(driver_test::sameBits(paused.driver().state(), frozen));
        CHECK_FALSE(paused.dirty());
        REQUIRE(paused.start().ok);
        CHECK(paused.generation() == generation);   // the same driver continues
        CHECK(&paused.driver() == driverPtr);
        runFrames(paused, 25);

        CHECK(driver_test::sameBits(paused.driver().state(), straight.driver().state()));
        CHECK(paused.driver().targetTime() == straight.driver().targetTime());
    }

    TEST_CASE("editing is blocked while Running and the scene does not change")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(2), err));
        const std::uint64_t domus = idOf(rc.editor().scene(), "Domus");
        REQUIRE(rc.start().ok);
        runFrames(rc, 5);

        const Scene before = rc.editor().scene();
        const std::uint64_t rev = rc.editor().revision();
        EditorModel& ed = rc.editor();
        CHECK(ed.locked());
        EditResult r = ed.addBody(BodyKind::Planet);
        CHECK_FALSE(r.ok);
        CHECK(r.error == kLockedMessage);
        CHECK_FALSE(ed.setFieldText(domus, Field::Radius, "1 km").ok);
        CHECK_FALSE(ed.remove(domus).ok);
        CHECK_FALSE(ed.setMu(domus, 1.0e12).ok);
        CHECK_FALSE(ed.addShuttle().ok);
        CHECK_FALSE(ed.setOrbitAround(900, domus, 1.0e6).ok);
        CHECK_FALSE(rc.applyEdits().ok);
        CHECK(sameScene(ed.scene(), before));
        CHECK(ed.revision() == rev);
        CHECK(ed.select(domus));   // selecting is not an edit
        CHECK(ed.selected() == domus);

        REQUIRE(rc.pause().ok);
        CHECK_FALSE(ed.locked());
        CHECK(ed.setRadius(domus, 5.0e5).ok);   // fine now
        REQUIRE(rc.start().ok);
        CHECK(ed.locked());
        CHECK_FALSE(ed.setRadius(domus, 4.0e5).ok);
        REQUIRE(rc.stop().ok);
        CHECK_FALSE(ed.locked());
        CHECK(ed.setRadius(domus, 4.0e5).ok);
    }

    TEST_CASE("advance only moves the simulation while Running")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(1), err));
        rc.driver().warp().set(1.0e3);
        CHECK_FALSE(rc.advance(0.1));
        CHECK(rc.driver().targetTime() == 0.0);
        REQUIRE(rc.start().ok);
        CHECK(rc.advance(0.1));
        CHECK(rc.driver().targetTime() > 0.0);
        REQUIRE(rc.pause().ok);
        const double t = rc.driver().targetTime();
        CHECK_FALSE(rc.advance(0.1));
        CHECK(rc.driver().targetTime() == t);
        CHECK(rc.driver().paused());
        REQUIRE(rc.start().ok);
        CHECK_FALSE(rc.driver().paused());
        CHECK(rc.advance(0.1));
        CHECK(rc.driver().targetTime() > t);
    }

    TEST_CASE("a single body runs without trouble")
    {
        RunControl rc(options());
        const std::uint64_t id = rc.editor().addBody(BodyKind::Planet, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}).id;
        REQUIRE(rc.applyEdits().ok);
        REQUIRE(rc.start().ok);
        runFrames(rc, 10);
        const DisplayBody* b = driver_test::findBody(rc.driver().state(), id);
        REQUIRE(b != nullptr);
        CHECK(b->r.x > 1.0);
        CHECK(b->v.x == doctest::Approx(10.0));
        REQUIRE(rc.stop().ok);
        const DisplayBody* b0 = driver_test::findBody(rc.driver().state(), id);
        REQUIRE(b0 != nullptr);
        CHECK(b0->r.x == 0.0);
    }

    TEST_CASE("loadScene replaces the document, stops the run, and keeps the warp")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(1), err));
        rc.driver().warp().set(100.0);
        REQUIRE(rc.start().ok);
        runFrames(rc, 3);
        const std::uint64_t gen = rc.generation();

        Scene empty;
        empty.name = "Other";
        REQUIRE(rc.loadScene(empty, err));
        CHECK(rc.state() == RunState::Stopped);
        CHECK(rc.generation() > gen);
        CHECK(rc.snapshot() == nullptr);
        CHECK(rc.editor().scene().name == "Other");
        CHECK_FALSE(rc.editor().locked());
        CHECK(rc.driver().warp().value() == 100.0);
        CHECK(rc.driver().state().bodies.empty());

        // A scene that cannot be resolved is refused and nothing changes
        Scene bad = lucernaWithShuttles(0);
        bad.bodies[1].orbit->parent = 4242;
        const std::uint64_t gen2 = rc.generation();
        CHECK_FALSE(rc.loadScene(bad, err));
        CHECK_FALSE(err.empty());
        CHECK(rc.editor().scene().name == "Other");
        CHECK(rc.generation() == gen2);
        CHECK(rc.lastError() == err);
    }

    TEST_CASE("Stop does not roll back the name and ID counters")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(1), err));
        REQUIRE(rc.start().ok);
        runFrames(rc, 3);
        REQUIRE(rc.pause().ok);
        const EditResult added = rc.editor().addBody(BodyKind::Planet);
        REQUIRE(added.ok);
        CHECK(rc.editor().body(added.id)->name == "Planet 1");
        REQUIRE(rc.stop().ok);
        CHECK(rc.editor().body(added.id) == nullptr);   // discarded with the run

        const EditResult again = rc.editor().addBody(BodyKind::Planet);
        REQUIRE(again.ok);
        CHECK(rc.editor().body(again.id)->name == "Planet 2");   // Planet 1 is not handed out again
        CHECK(again.id > added.id);                              // nor its ID
    }

    TEST_CASE("the object list follows the run")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(collisionScene(), err));
        CHECK(rc.listObjects().size() == 5);   // 3 bodies and 2 shuttles
        rc.driver().warp().set(10.0);
        REQUIRE(rc.start().ok);
        REQUIRE(runUntilMergeAndLoss(rc));
        const std::vector<ObjectRow> live = rc.listObjects();
        std::size_t bodies = 0, shuttles = 0;
        for (const ObjectRow& r : live) (r.type == ObjectType::Body ? bodies : shuttles)++;
        CHECK(bodies == 2);     // the merged pair is one body
        CHECK(shuttles == 1);   // the doomed shuttle is gone
        REQUIRE(rc.stop().ok);
        CHECK(rc.listObjects().size() == 5);
    }
}
