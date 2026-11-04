#include <doctest.h>
#include <filesystem>
#include <fstream>
#include "app/presets.h"
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

// Review fixes of M6: saving never overwrites, settings survive rebuilds and the Stopped capture, Start needs bodies when
// paused too, the look-ahead events cut off by an edit are handed over.
TEST_SUITE("app m6 review")
{
    TEST_CASE("uniqueScenePath never names an existing file")
    {
        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / "awl_cosmos_unique_scene_test";
        fs::remove_all(dir);
        fs::create_directories(dir);
        const std::string base = (dir / "my_scene").generic_string();
        CHECK(uniqueScenePath(base) == base + ".json");   // free: unchanged, extension added
        std::ofstream(base + ".json") << "{}";
        CHECK(uniqueScenePath(base + ".json") == base + "_2.json");
        std::ofstream(base + "_2.json") << "{}";
        CHECK(uniqueScenePath(base + ".json") == base + "_3.json");
        CHECK(uniqueScenePath(base + "_2.json") == base + "_3.json");   // not my_scene_2_2
        fs::remove_all(dir);
    }

    TEST_CASE("Start from Paused refuses a scene without bodies")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(0), err));
        REQUIRE(rc.start().ok);
        runFrames(rc, 3);
        REQUIRE(rc.pause().ok);
        for (int guard = 0; guard < 100 && !rc.editor().scene().bodies.empty(); ++guard) {
            REQUIRE(rc.editor().select(rc.editor().scene().bodies.back().id));
            REQUIRE(rc.editor().remove(rc.editor().selected()).ok);
        }
        REQUIRE(rc.editor().scene().bodies.empty());
        const EditResult r = rc.start();
        CHECK_FALSE(r.ok);
        CHECK(rc.state() == RunState::Paused);
    }

    TEST_CASE("the Simulation window's settings survive a rebuild and reach the Stopped capture")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(1), err));
        rc.driver().sim().setInfiniteFuel(true);
        rc.driver().sim().setControllerTick(0.25);

        Scene stopped;
        REQUIRE(rc.captureCurrent(stopped));
        CHECK(stopped.settings.infiniteFuel);
        CHECK(stopped.settings.controllerTick_s == 0.25);

        REQUIRE(rc.start().ok);   // rebuilds from the scene
        CHECK(rc.driver().sim().infiniteFuel());
        CHECK(rc.driver().sim().controllerTick() == 0.25);
        runFrames(rc, 3);
        REQUIRE(rc.pause().ok);
        REQUIRE(rc.editor().setFieldText(rc.editor().scene().bodies.front().id, Field::Name, "Renamed").ok);
        REQUIRE(rc.applyEdits().ok);
        CHECK(rc.driver().sim().infiniteFuel());
        CHECK(rc.driver().sim().controllerTick() == 0.25);
    }

    TEST_CASE("an edit while paused hands over the events of the physics look-ahead")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(collisionScene(), err));
        REQUIRE(rc.start().ok);
        REQUIRE(runUntilMergeAndLoss(rc));
        const std::size_t pending = rc.driver().pendingEventCount();
        REQUIRE(pending > 0);
        REQUIRE(rc.pause().ok);
        CHECK(rc.takeFlushedEvents().empty());   // nothing is cut off before an edit
        REQUIRE(rc.editor().setFieldText(3, Field::Name, "Renamed C").ok);
        REQUIRE(rc.applyEdits().ok);
        const std::vector<Event> cut = rc.takeFlushedEvents();
        CHECK(cut.size() == pending);
        CHECK(rc.takeFlushedEvents().empty());   // handed over once
    }
}
