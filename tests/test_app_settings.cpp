#include <doctest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include "app/format.h"
#include "app/settings.h"
#include "app/units.h"

using namespace cosmos::app;

// settings.json (V1.1 step 1): defaults, round trip, tolerance of missing / unknown / wrong-typed keys and of a corrupt file,
// clamping, and the display preferences the readouts follow.

namespace {

std::string tempPath(const char* name)
{
    return (std::filesystem::temp_directory_path() / (std::string("awl_cosmos_test_") + name)).string();
}

void writeFile(const std::string& path, const std::string& text)
{
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << text;
}

// A settings value with every field away from its default
Settings unusual()
{
    Settings s;
    s.vsync = false;
    s.msaa = 8;
    s.uiScale = 1.5f;
    s.fpsCap = 144;
    s.starQuality = Quality::High;
    s.skyboxQuality = Quality::Low;
    s.threads = 6;
    s.defaultWarp = 1000.0;
    s.autoSlowDown = false;
    s.slowDownAnyShuttle = true;
    s.controllerTick = 0.25;
    s.lengthUnit = LengthUnit::AU;
    s.massUnit = MassUnit::SolarMasses;
    s.dateFormat = DateFormat::YearDay;
    s.overlays.names = false;
    s.overlays.trails = true;
    s.overlays.hill = true;
    s.overlays.atmospheres = true;
    return s;
}

// Both sides serialised: equal text means every field is equal
bool same(const Settings& a, const Settings& b) { return settingsToJson(a) == settingsToJson(b); }

// Resets the process-wide display preference when a test is done
struct PrefsGuard {
    ~PrefsGuard() { displayPrefs() = DisplayPrefs(); }
};

} // namespace

TEST_SUITE("app settings")
{
    TEST_CASE("defaults are inside their own ranges")
    {
        Settings s;
        const std::string before = settingsToJson(s);
        clampSettings(s);
        CHECK(settingsToJson(s) == before);
        CHECK(s.vsync);
        CHECK(s.msaa == 4);
        CHECK(s.uiScale == 0.0f);
        CHECK(s.fpsCap == 0);
        CHECK(s.threads == 0);
        CHECK(s.defaultWarp == 1.0);
        CHECK(s.autoSlowDown);
        CHECK_FALSE(s.slowDownAnyShuttle);
        CHECK(s.controllerTick == doctest::Approx(0.1));
        CHECK(s.overlays.names);
        CHECK_FALSE(s.overlays.trails);
    }

    TEST_CASE("JSON round trip keeps every field")
    {
        const Settings s = unusual();
        bool ok = false;
        const Settings back = settingsFromJson(settingsToJson(s), &ok);
        CHECK(ok);
        CHECK(same(back, s));
        CHECK(back.msaa == 8);
        CHECK(back.fpsCap == 144);
        CHECK(back.uiScale == doctest::Approx(1.5f));
        CHECK(back.starQuality == Quality::High);
        CHECK(back.skyboxQuality == Quality::Low);
        CHECK(back.lengthUnit == LengthUnit::AU);
        CHECK(back.massUnit == MassUnit::SolarMasses);
        CHECK(back.dateFormat == DateFormat::YearDay);
        CHECK_FALSE(back.overlays.names);
        CHECK(back.overlays.trails);
        CHECK(back.overlays.hill);
        CHECK(back.overlays.atmospheres);
        CHECK_FALSE(back.overlays.soi);
    }

    TEST_CASE("file round trip, and a missing file gives the defaults")
    {
        const std::string path = tempPath("roundtrip.json");
        std::filesystem::remove(path);
        Settings loaded = unusual();
        std::string note = "stale";
        CHECK(loadSettings(path, loaded, note));   // no file yet is not an error
        CHECK(note.empty());
        CHECK(same(loaded, Settings()));

        std::string error;
        REQUIRE(saveSettings(path, unusual(), error));
        CHECK(error.empty());
        CHECK(loadSettings(path, loaded, note));
        CHECK(same(loaded, unusual()));
        std::filesystem::remove(path);
    }

    TEST_CASE("saving into a folder that does not exist reports the error")
    {
        std::string error;
        CHECK_FALSE(saveSettings((std::filesystem::temp_directory_path() / "awl_cosmos_no_such_dir" / "x" / "s.json").string(), Settings(), error));
        CHECK_FALSE(error.empty());
    }

    TEST_CASE("missing keys keep the defaults")
    {
        CHECK(same(settingsFromJson("{}"), Settings()));
        const Settings s = settingsFromJson(R"({"graphics": {"vsync": false}, "display": {"overlays": {"roche": true}}})");
        CHECK_FALSE(s.vsync);
        CHECK(s.msaa == 4);              // not mentioned: default
        CHECK(s.overlays.roche);
        CHECK(s.overlays.names);         // sibling key of the overlays object: default
        CHECK(s.defaultWarp == 1.0);     // a whole section missing
    }

    TEST_CASE("unknown keys, wrong types and bad names are ignored")
    {
        const Settings s = settingsFromJson(R"({
            "version": 99, "futureThing": {"a": 1},
            "graphics": {"vsync": "yes", "msaa": "lots", "fpsCap": 120, "starQuality": "ultra", "skyboxQuality": 7, "extra": true},
            "simulation": {"threads": [1, 2], "defaultWarp": null, "autoSlowDown": 0, "controllerTick": 0.5},
            "display": {"lengthUnit": "parsecs", "massUnit": "kg", "dateFormat": 3, "overlays": "all"}
        })");
        CHECK(s.vsync);                           // wrong type: default
        CHECK(s.msaa == 4);
        CHECK(s.fpsCap == 120);                   // the readable keys of the same object still count
        CHECK(s.starQuality == Quality::Medium);  // unknown name: default
        CHECK(s.skyboxQuality == Quality::Medium);
        CHECK(s.threads == 0);
        CHECK(s.defaultWarp == 1.0);
        CHECK(s.autoSlowDown);
        CHECK(s.controllerTick == doctest::Approx(0.5));
        CHECK(s.lengthUnit == LengthUnit::Auto);
        CHECK(s.massUnit == MassUnit::Kilograms);
        CHECK(s.dateFormat == DateFormat::DayClock);
        CHECK(s.overlays.names);                  // "overlays" is not an object: defaults
    }

    TEST_CASE("a corrupt or unusable file gives the defaults and never throws")
    {
        const char* bad[] = {"", "   ", "not json at all", "{\"graphics\": {\"vsync\": fal", "[1, 2, 3]", "42", "null", "\"text\"",
                             "{\"graphics\": ", "\xff\xfe\x00\x01"};
        for (const char* text : bad) {
            bool ok = true;
            CHECK_NOTHROW(settingsFromJson(text, &ok));
            CHECK_FALSE(ok);
            CHECK(same(settingsFromJson(text), Settings()));
        }

        const std::string path = tempPath("corrupt.json");
        writeFile(path, "{ this is { not json");
        Settings loaded = unusual();
        std::string note;
        CHECK_FALSE(loadSettings(path, loaded, note));
        CHECK_FALSE(note.empty());
        CHECK(same(loaded, Settings()));
        std::filesystem::remove(path);

        writeFile(path, "");   // an empty file (a crash while saving)
        CHECK_FALSE(loadSettings(path, loaded, note));
        CHECK(same(loaded, Settings()));
        std::filesystem::remove(path);
    }

    TEST_CASE("merging JSON changes only the keys it names")
    {
        Settings s = unusual();
        CHECK(mergeSettingsJson(s, R"({"display": {"lengthUnit": "km"}})"));
        CHECK(s.lengthUnit == LengthUnit::Kilometres);
        CHECK(s.massUnit == MassUnit::SolarMasses);   // untouched
        CHECK(s.msaa == 8);
        const std::string before = settingsToJson(s);
        CHECK_FALSE(mergeSettingsJson(s, "garbage"));
        CHECK(settingsToJson(s) == before);
    }

    TEST_CASE("values are clamped into range")
    {
        Settings s;
        s.msaa = 3;                       // between 2 and 4: ties go up
        s.fpsCap = 100;                   // nearer to 120 than 60
        s.uiScale = 9.0f;
        s.threads = 99;
        s.defaultWarp = 1.0e9;
        s.controllerTick = 0.0;
        clampSettings(s);
        CHECK(s.msaa == 4);
        CHECK(s.fpsCap == 120);
        CHECK(s.uiScale == kMaxUiScale);
        CHECK(s.threads == kMaxThreadSetting);
        CHECK(s.defaultWarp == kMaxDefaultWarp);
        CHECK(s.controllerTick == kMinControllerTick);

        s = Settings();
        s.msaa = -5;
        s.fpsCap = 1000;
        s.uiScale = 0.1f;                 // below the minimum but not "automatic"
        s.threads = -3;
        s.defaultWarp = 0.0;
        s.controllerTick = 1.0e6;
        clampSettings(s);
        CHECK(s.msaa == 0);
        CHECK(s.fpsCap == 240);
        CHECK(s.uiScale == kMinUiScale);
        CHECK(s.threads == 0);
        CHECK(s.defaultWarp == kMinDefaultWarp);
        CHECK(s.controllerTick == kMaxControllerTick);

        s = Settings();
        s.uiScale = -1.0f;                // zero or less is automatic
        clampSettings(s);
        CHECK(s.uiScale == 0.0f);
    }

    TEST_CASE("non-finite numbers become the defaults")
    {
        Settings s;
        s.uiScale = std::numeric_limits<float>::quiet_NaN();
        s.defaultWarp = std::numeric_limits<double>::infinity();
        s.controllerTick = std::numeric_limits<double>::quiet_NaN();
        clampSettings(s);
        CHECK(s.uiScale == 0.0f);
        CHECK(s.defaultWarp == 1.0);
        CHECK(s.controllerTick == doctest::Approx(0.1));
    }

    TEST_CASE("out-of-range values in a file are clamped on load")
    {
        const Settings s = settingsFromJson(R"({"graphics": {"msaa": 16, "fpsCap": 75, "uiScale": 100}, "simulation": {"threads": 1000, "defaultWarp": -4, "controllerTick": 0}})");
        CHECK(s.msaa == 8);
        CHECK(s.fpsCap == 60);   // 75 is 15 from 60 and 45 from 120
        CHECK(s.uiScale == kMaxUiScale);
        CHECK(s.threads == kMaxThreadSetting);
        CHECK(s.defaultWarp == kMinDefaultWarp);
        CHECK(s.controllerTick == kMinControllerTick);
    }
}

TEST_SUITE("app display preferences")
{
    TEST_CASE("the default preferences leave the readouts as they were")
    {
        CHECK(formatDistance(1.496e11) == "1.00 AU");
        CHECK(formatDistance(6.371e6) == "6.37 Mm");
        CHECK(formatSimDate(90061.0) == "Day 1  01:01:01");
        CHECK(formatQuantity(1.5e9, Quantity::Length, nullptr, 3) == "1.5 Gm");
    }

    TEST_CASE("a fixed length unit applies to distances and fields")
    {
        PrefsGuard guard;
        Settings s;
        s.lengthUnit = LengthUnit::Kilometres;
        applyDisplayPrefs(s);
        CHECK(formatDistance(1.0e9) == "1000000 km");
        CHECK(formatQuantity(6.371e6, Quantity::Length, nullptr, 4) == "6371 km");
        CHECK(formatQuantity(6.371e6, Quantity::Length, "m", 4) == "6.371e6 m");   // an explicit unit still wins

        s.lengthUnit = LengthUnit::AU;
        applyDisplayPrefs(s);
        CHECK(formatDistance(1.495978707e11) == "1.00 AU");
        CHECK(formatQuantity(1.495978707e11, Quantity::Length, nullptr, 4) == "1 AU");

        s.lengthUnit = LengthUnit::Metres;
        applyDisplayPrefs(s);
        CHECK(formatDistance(1500.0) == "1500 m");
        CHECK(formatQuantity(1500.0, Quantity::Length, nullptr, 4) == "1500 m");
    }

    TEST_CASE("a fixed mass unit applies to body masses and reads back")
    {
        PrefsGuard guard;
        const double muEarth = gmEarth();
        Settings s;
        s.massUnit = MassUnit::EarthMasses;
        applyDisplayPrefs(s);
        CHECK(formatQuantity(muEarth, Quantity::BodyMass, nullptr, 4) == "1 M_earth");
        CHECK(formatQuantity(gmSun(), Quantity::BodyMass, nullptr, 6) == "332946 M_earth");   // Sun / Earth, 6 figures

        s.massUnit = MassUnit::SolarMasses;
        applyDisplayPrefs(s);
        CHECK(formatQuantity(gmSun(), Quantity::BodyMass, nullptr, 4) == "1 M_sun");

        s.massUnit = MassUnit::Kilograms;
        applyDisplayPrefs(s);
        const std::string text = formatQuantity(muEarth, Quantity::BodyMass, nullptr, 6);
        CHECK(text.find(" kg") != std::string::npos);
        const ParseResult back = parseQuantity(text, Quantity::BodyMass);   // every unit still parses
        REQUIRE(back.ok);
        CHECK(back.value == doctest::Approx(muEarth).epsilon(1.0e-4));
    }

    TEST_CASE("date formats")
    {
        const double t = 400.0 * 86400.0 + 3.0 * 3600.0 + 4.0 * 60.0 + 5.0;   // day 400, 03:04:05
        CHECK(formatSimDate(t, DateFormat::DayClock) == "Day 400  03:04:05");
        CHECK(formatSimDate(t, DateFormat::DayOnly) == "Day 400");
        CHECK(formatSimDate(t, DateFormat::YearDay) == "Year 1, Day 35  03:04:05");
        CHECK(formatSimDate(-1.0, DateFormat::YearDay) == "-Year 0, Day 0  00:00:01");
        CHECK(formatSimDate(std::numeric_limits<double>::quiet_NaN(), DateFormat::YearDay) == "Day --");

        PrefsGuard guard;
        Settings s;
        s.dateFormat = DateFormat::DayOnly;
        applyDisplayPrefs(s);
        CHECK(formatSimDate(t) == "Day 400");
    }
}
