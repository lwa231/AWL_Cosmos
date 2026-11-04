#include <doctest.h>
#include <filesystem>
#include <fstream>
#include "app/presets.h"
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

namespace {

std::string sceneDir() { return std::string(COSMOS_SOURCE_DIR) + "/scenes"; }

std::string tempDir(const char* leaf)
{
    const std::filesystem::path p = std::filesystem::temp_directory_path() / (std::string("awl_cosmos_test_") + leaf);
    std::filesystem::remove_all(p);
    return p.string();
}

} // namespace

// Presets (plan 11.2) and user scene files
TEST_SUITE("app runcontrol presets")
{
    TEST_CASE("the preset list: Empty sandbox, then Lucerna")
    {
        const std::vector<PresetInfo>& list = presets();
        REQUIRE(list.size() >= 2);
        CHECK(list[0].id == "empty");
        CHECK(list[0].name == "Empty sandbox");
        CHECK(list[1].id == "lucerna");
        CHECK(list[1].name == "Lucerna");
        for (const PresetInfo& p : list) {
            CHECK_FALSE(p.name.empty());
            CHECK_FALSE(p.description.empty());
            CHECK(presetAvailable(p, sceneDir()));
        }
        CHECK(findPreset("lucerna") != nullptr);
        CHECK(findPreset("nope") == nullptr);
        CHECK(std::string(defaultSceneDir()) == "scenes");
    }

    TEST_CASE("Empty sandbox is empty; every body added is named <Kind> <n>")
    {
        Scene s;
        std::string err;
        REQUIRE(loadPreset("empty", sceneDir(), s, err));
        CHECK(s.bodies.empty());
        CHECK(s.shuttles.empty());
        CHECK(s.name == "Empty sandbox");
        CHECK(s.epoch_s == 0.0);

        RunControl rc(options());
        REQUIRE(rc.loadScene(s, err));
        CHECK(rc.editor().addBody(BodyKind::Star).ok);
        CHECK(rc.editor().addBody(BodyKind::Planet).ok);
        CHECK(rc.editor().addBody(BodyKind::Planet).ok);
        CHECK(rc.editor().scene().bodies[0].name == "Star 1");
        CHECK(rc.editor().scene().bodies[1].name == "Planet 1");
        CHECK(rc.editor().scene().bodies[2].name == "Planet 2");
        // The shipped scenes/empty.json agrees with the built-in one
        Scene file;
        REQUIRE(loadSceneFile(sceneDir() + "/empty.json", file, err));
        CHECK(sameScene(file, emptyScene()));
    }

    TEST_CASE("Lucerna loads from scenes/lucerna.json and starts")
    {
        Scene s;
        std::string err;
        REQUIRE_MESSAGE(loadPreset("lucerna", sceneDir(), s, err), err);
        CHECK(s.name == "Lucerna");
        CHECK(s.bodies.size() == 14);
        CHECK_FALSE(s.shuttles.empty());

        RunControl rc(options());
        REQUIRE(rc.loadScene(s, err));
        // Template names consume no counter
        CHECK(rc.editor().namesIssued(BodyKind::Planet) == 0);
        CHECK(rc.editor().addBody(BodyKind::Planet).ok);
        CHECK(rc.editor().scene().bodies.back().name == "Planet 1");
        REQUIRE(rc.editor().remove(rc.editor().selected()).ok);
        REQUIRE(rc.start().ok);
        runFrames(rc, 5);
        CHECK(rc.driver().targetTime() > 0.0);
    }

    TEST_CASE("unknown ids and missing files are errors")
    {
        Scene s;
        std::string err;
        CHECK_FALSE(loadPreset("atlantis", sceneDir(), s, err));
        CHECK(err.find("atlantis") != std::string::npos);
        err.clear();
        CHECK_FALSE(loadPreset("lucerna", sceneDir() + "/missing_folder", s, err));
        CHECK_FALSE(err.empty());
    }

    TEST_CASE("the extension point: a preset can be registered, a missing file shows as unavailable")
    {
        PresetInfo p;
        p.id = "test_solar";
        p.name = "Test system";
        p.description = "registered by a test";
        p.file = "test_solar_system_missing.json";
        registerPreset(p);
        const PresetInfo* found = findPreset("test_solar");
        REQUIRE(found != nullptr);
        CHECK_FALSE(presetAvailable(*found, sceneDir()));
        Scene s;
        std::string err;
        CHECK_FALSE(loadPreset("test_solar", sceneDir(), s, err));

        // A built-in generator
        PresetInfo b;
        b.id = "test_solar";   // replaces
        b.name = "Test system";
        b.description = "built in";
        b.build = [] {
            Scene t;
            t.name = "Built";
            return t;
        };
        registerPreset(b);
        std::size_t count = 0;
        for (const PresetInfo& q : presets()) count += q.id == "test_solar" ? 1 : 0;
        CHECK(count == 1);
        REQUIRE(loadPreset("test_solar", sceneDir(), s, err));
        CHECK(s.name == "Built");
    }

    TEST_CASE("a scene built from Empty is saved, reloaded and gives the same scene and the same run")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(emptyScene(), err));
        EditorModel& ed = rc.editor();
        const std::uint64_t star = ed.addBody(BodyKind::Star).id;
        const std::uint64_t planet = ed.addBody(BodyKind::Planet).id;
        const std::uint64_t moon = ed.addBody(BodyKind::Moon).id;
        REQUIRE(ed.setOrbitAround(planet, star, 1.0e11).ok);
        REQUIRE(ed.setOrbitAround(moon, planet, 4.0e8).ok);
        REQUIRE(ed.setFieldText(planet, Field::OrbitE, "0.02").ok);
        REQUIRE(ed.setFieldText(planet, Field::Name, "Terra").ok);
        REQUIRE(ed.enableAtmosphere(planet, true).ok);
        REQUIRE(ed.setSpinPeriodHours(planet, 24.0).ok);
        const std::uint64_t shuttle = ed.addShuttle(ControllerKind::Prograde).id;
        REQUIRE(ed.setOrbitAround(shuttle, planet, 7.0e6).ok);
        REQUIRE(ed.setName(star, "Sol").ok);
        REQUIRE(rc.applyEdits().ok);

        const std::string dir = tempDir("roundtrip");
        const std::string path = dir + "/my scene";
        REQUIRE_MESSAGE(saveSceneToPath(path, ed.scene(), err), err);
        CHECK(withSceneExtension(path) == path + ".json");
        CHECK(withSceneExtension("a.scene") == "a.scene");
        REQUIRE(std::filesystem::exists(path + ".json"));
        CHECK(listSceneFiles(dir) == std::vector<std::string>{"my scene.json"});

        Scene loaded;
        REQUIRE_MESSAGE(loadSceneFromPath(path + ".json", loaded, err), err);
        CHECK(sameScene(loaded, ed.scene()));

        // Loading it into a fresh control gives the same start and, after a run, the same state
        RunControl other(options());
        REQUIRE(other.loadScene(loaded, err));
        CHECK(driver_test::sameBits(other.driver().state(), rc.driver().state()));
        rc.driver().warp().set(1.0e5);
        other.driver().warp().set(1.0e5);
        REQUIRE(rc.start().ok);
        REQUIRE(other.start().ok);
        runFrames(rc, 20);
        runFrames(other, 20);
        CHECK(driver_test::sameBits(other.driver().state(), rc.driver().state()));
        // Names continue past the generic ones in the file (the counters are rebuilt from the names, so a number whose
        // body was renamed before saving is free again after loading)
        CHECK(other.editor().namesIssued(BodyKind::Moon) == 1);
        CHECK(other.editor().namesIssued(BodyKind::Planet) == 0);
        CHECK(other.editor().shuttleNamesIssued() == 1);
        std::filesystem::remove_all(dir);
    }

    TEST_CASE("a scene without a name takes the file name; invalid files are refused with a message")
    {
        const std::string dir = tempDir("names");
        std::filesystem::create_directories(dir);
        {
            std::ofstream f(dir + "/unnamed.json");
            f << R"({"bodies": [], "shuttles": []})";
        }
        {
            std::ofstream f(dir + "/broken.json");
            f << R"({"bodies": [ {"id": 1, "mu": -5, "radius": 10} ]})";
        }
        {
            std::ofstream f(dir + "/notjson.json");
            f << "this is not json";
        }
        {
            std::ofstream f(dir + "/readme.txt");
            f << "ignored";
        }
        Scene s;
        std::string err;
        REQUIRE_MESSAGE(loadSceneFromPath(dir + "/unnamed.json", s, err), err);
        CHECK(s.name == "unnamed");
        CHECK_FALSE(loadSceneFromPath(dir + "/broken.json", s, err));
        CHECK_FALSE(err.empty());
        err.clear();
        CHECK_FALSE(loadSceneFromPath(dir + "/notjson.json", s, err));
        CHECK_FALSE(err.empty());
        err.clear();
        CHECK_FALSE(loadSceneFromPath(dir + "/absent.json", s, err));
        CHECK_FALSE(err.empty());
        const std::vector<std::string> names = listSceneFiles(dir);
        CHECK(names == std::vector<std::string>{"broken.json", "notjson.json", "unnamed.json"});
        CHECK(listSceneFiles(dir + "/nothing_here").empty());
        std::filesystem::remove_all(dir);
    }
}
