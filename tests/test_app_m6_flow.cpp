#include <doctest.h>
#include <cmath>
#include <filesystem>
#include <string>
#include "app/presets.h"
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

// The M6 done-when (plan section 15), driven the way the left panel and the bottom bar drive it: build a scene from the empty
// sandbox with typed fields, run it, Stop (restores), Save current, load it again, Pause / edit / Resume.
TEST_SUITE("app m6 flow")
{
    namespace {
    std::string tempPath(const char* leaf)
    {
        const std::filesystem::path dir = std::filesystem::temp_directory_path() / "awl_cosmos_test_m6_flow";
        std::filesystem::create_directories(dir);
        return (dir / leaf).string();
    }

    const DisplayBody* find(const DisplayState& st, const std::string& name)
    {
        for (const DisplayBody& b : st.bodies)
            if (b.name == name) return &b;
        return nullptr;
    }

    double distance(const DisplayBody& a, const DisplayBody& b) { return std::hypot(a.r.x - b.r.x, a.r.y - b.r.y); }
    } // namespace

    TEST_CASE("build from Empty, Start, Stop restores, Save current, reload, Pause and Resume")
    {
        RunControl rc(options());
        Scene empty;
        std::string err;
        REQUIRE(loadPreset("empty", defaultSceneDir(), empty, err));
        REQUIRE(rc.loadScene(empty, err));
        EditorModel& ed = rc.editor();
        CHECK(rc.state() == RunState::Stopped);
        CHECK(rc.start().ok == false);   // nothing to run yet
        CHECK_FALSE(rc.lastError().empty());

        // a star, and a planet in orbit around it at 1 AU, set through the same text fields the panel uses
        const EditResult star = ed.addBody(BodyKind::Star);
        REQUIRE(star.ok);
        const EditResult planet = ed.addBody(BodyKind::Planet);
        REQUIRE(planet.ok);
        CHECK(ed.body(star.id)->name == "Star 1");
        CHECK(ed.body(planet.id)->name == "Planet 1");
        REQUIRE(ed.setOrbitAround(planet.id, star.id, 0.0 + 1.495978707e11).ok);
        REQUIRE(ed.setFieldText(planet.id, Field::Mass, "2 M_earth").ok);
        REQUIRE(ed.setFieldText(planet.id, Field::OrbitE, "0.1").ok);
        CHECK_FALSE(ed.setFieldText(planet.id, Field::Radius, "abc").ok);   // the error is shown next to the field, nothing changes

        rc.advance(0.0);   // not Running: applies the pending edits
        REQUIRE_FALSE(rc.dirty());
        const Scene before = ed.scene();
        const DisplayState stateBefore = rc.driver().state();
        REQUIRE(stateBefore.bodies.size() == 2);

        // Start: snapshot, the editor locks, the run advances
        REQUIRE(rc.start().ok);
        CHECK(rc.state() == RunState::Running);
        CHECK(ed.locked());
        CHECK_FALSE(ed.setFieldText(planet.id, Field::Name, "Terra").ok);   // fields are disabled while Running
        CHECK(ed.setFieldText(planet.id, Field::Name, "Terra").error == kLockedMessage);
        rc.driver().warp().set(1.0e5);
        for (int i = 0; i < 60; ++i) REQUIRE(rc.advance(1.0 / 60.0));
        const DisplayState running = rc.driver().state();
        CHECK(running.t > stateBefore.t);
        const DisplayBody* p = find(running, "Planet 1");
        const DisplayBody* s = find(running, "Star 1");
        REQUIRE((p && s));
        CHECK(distance(*p, *s) > 0.85 * 1.495978707e11);   // still on its orbit (e = 0.1 about 1 AU)
        CHECK(distance(*p, *s) < 1.2 * 1.495978707e11);

        // Save current while Running: a scene with explicit states that loads back and resolves
        const std::string live = tempPath("live.json");
        REQUIRE(rc.saveCurrent(live, err));

        // Stop: the scene is back exactly as at Start, bit for bit
        REQUIRE(rc.stop().ok);
        CHECK(rc.state() == RunState::Stopped);
        CHECK_FALSE(ed.locked());
        CHECK(sameScene(ed.scene(), before));
        CHECK(driver_test::sameBits(rc.driver().state(), stateBefore));

        // Save current while Stopped keeps the orbits; reload gives the same scene
        const std::string path = tempPath("built.json");
        REQUIRE(rc.saveCurrent(path, err));
        Scene loaded;
        REQUIRE(loadSceneFromPath(path, loaded, err));
        CHECK(sameScene(loaded, before));
        RunControl other(options());
        REQUIRE(other.loadScene(loaded, err));
        CHECK(driver_test::sameBits(other.driver().state(), stateBefore));

        // The live capture reloads as a scene of two bodies at the run's moment
        Scene captured;
        REQUIRE(loadSceneFromPath(live, captured, err));
        CHECK(captured.bodies.size() == 2);
        CHECK(captured.epoch_s > 0.0);
        RunControl third(options());
        REQUIRE(third.loadScene(captured, err));
        CHECK(third.driver().state().bodies.size() == 2);

        // Start, Pause (edit allowed), edit, Resume: the run continues from where it paused
        REQUIRE(rc.start().ok);
        for (int i = 0; i < 20; ++i) rc.advance(1.0 / 60.0);
        REQUIRE(rc.pause().ok);
        CHECK(rc.state() == RunState::Paused);
        CHECK_FALSE(ed.locked());
        const double pausedAt = rc.driver().simTime();
        REQUIRE(ed.setFieldText(planet.id, Field::Name, "Terra").ok);
        rc.advance(1.0 / 60.0);   // applies the edit (rebuilds the simulation at the pause moment)
        CHECK(rc.driver().simTime() == doctest::Approx(pausedAt).epsilon(1e-12));
        REQUIRE(rc.start().ok);
        for (int i = 0; i < 10; ++i) rc.advance(1.0 / 60.0);
        CHECK(rc.driver().simTime() > pausedAt);
        CHECK(find(rc.driver().state(), "Terra") != nullptr);

        // Stop: the snapshot of the second Start (the scene as built) is back, including the old name
        REQUIRE(rc.stop().ok);
        CHECK(sameScene(ed.scene(), before));
    }
}
