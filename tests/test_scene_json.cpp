#include <doctest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include "core/scene.h"
#include "core/scene_json.h"
#include "lucerna_data.h"
#include "test_scene_util.h"

using namespace cosmos;
using scene_test::Lucerna;
using scene_test::sceneFile;

namespace {

// The example of plan section 11.1, character for character
const char* const kPlanExample = R"JSON({
  "name": "Lucerna",
  "units": "SI",
  "epoch_s": 0,
  "bodies": [
    { "id": 1, "name": "Lucerna", "kind": "Star", "kindLocked": true, "fictionalDensity": true,
      "mu": 1.25e18, "radius": 2.4e8, "spinPeriod_h": 480, "colour": [1.0, 0.95, 0.82],
      "position": [0,0,0], "velocity": [0,0,0] },
    { "id": 4, "name": "Domus", "kind": "Planet", "kindLocked": true, "fictionalDensity": true,
      "mu": 3.767e12, "radius": 6.2e5, "spinPeriod_h": 5.8, "colour": [0.28, 0.48, 0.78],
      "atmosphere": { "rho0": 1.22, "scaleHeight": 7000 },
      "orbit": { "parent": 1, "a": 1.45e10, "e": 0.015, "argPeriapsis_deg": 103, "meanAnomaly_deg": 35, "retrograde": false } }
  ],
  "shuttles": [
    { "id": 1001, "name": "Shuttle 1", "controller": "Prograde",
      "orbit": { "parent": 4, "a": 7.2e5, "e": 0, "argPeriapsis_deg": 0, "meanAnomaly_deg": 0, "retrograde": false },
      "dryMass": 2000, "fuelMass": 3000, "maxThrust": 60000, "exhaustVelocity": 3000,
      "cdA": 2.0, "radius": 2.0, "maxTorque": 5000, "inertia": 10000 }
  ],
  "settings": { "infiniteFuel": false, "controllerTick_s": 0.1 }
})JSON";

// A minimal valid body to build malformed variants from
const char* const kBodyOk = R"JSON({ "id": 1, "name": "A", "mu": 1e12, "radius": 1000, "position": [0,0,0], "velocity": [0,0,0] })JSON";

std::string sceneWith(const std::string& bodies) { return "{ \"bodies\": [" + bodies + "] }"; }

bool sameOrbit(const std::optional<SceneOrbit>& a, const std::optional<SceneOrbit>& b)
{
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    return a->parent == b->parent && a->a == b->a && a->e == b->e && a->argPeriapsis_deg == b->argPeriapsis_deg &&
           a->meanAnomaly_deg == b->meanAnomaly_deg && a->retrograde == b->retrograde;
}

bool sameVec(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

// Exact (bitwise-equal doubles) comparison of two scenes; returns the first difference or an empty string
std::string firstDifference(const Scene& a, const Scene& b)
{
    if (a.name != b.name) return "name";
    if (a.units != b.units) return "units";
    if (a.epoch_s != b.epoch_s) return "epoch_s";
    if (a.settings.infiniteFuel != b.settings.infiniteFuel) return "settings.infiniteFuel";
    if (a.settings.controllerTick_s != b.settings.controllerTick_s) return "settings.controllerTick_s";
    if (a.bodies.size() != b.bodies.size()) return "body count";
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const SceneBody& x = a.bodies[i];
        const SceneBody& y = b.bodies[i];
        const std::string at = "bodies[" + std::to_string(i) + "].";
        if (x.id != y.id) return at + "id";
        if (x.name != y.name) return at + "name";
        if (x.kind != y.kind) return at + "kind";
        if (x.kindLocked != y.kindLocked) return at + "kindLocked";
        if (x.fictionalDensity != y.fictionalDensity) return at + "fictionalDensity";
        if (x.mu != y.mu) return at + "mu";
        if (x.radius != y.radius) return at + "radius";
        if (x.spinPeriod_h != y.spinPeriod_h) return at + "spinPeriod_h";
        if (x.inertiaFactor != y.inertiaFactor) return at + "inertiaFactor";
        if (x.atmosphere.has_value() != y.atmosphere.has_value()) return at + "atmosphere";
        if (x.atmosphere && (x.atmosphere->rho0 != y.atmosphere->rho0 || x.atmosphere->scaleHeight != y.atmosphere->scaleHeight))
            return at + "atmosphere values";
        if (x.colour != y.colour) return at + "colour";
        if (!sameOrbit(x.orbit, y.orbit)) return at + "orbit";
        if (!x.orbit && (!sameVec(x.position, y.position) || !sameVec(x.velocity, y.velocity))) return at + "state";
    }
    if (a.shuttles.size() != b.shuttles.size()) return "shuttle count";
    for (std::size_t i = 0; i < a.shuttles.size(); ++i) {
        const SceneShuttle& x = a.shuttles[i];
        const SceneShuttle& y = b.shuttles[i];
        const std::string at = "shuttles[" + std::to_string(i) + "].";
        if (x.id != y.id || x.name != y.name || x.controller != y.controller) return at + "identity";
        if (!sameOrbit(x.orbit, y.orbit)) return at + "orbit";
        if (!x.orbit && (!sameVec(x.position, y.position) || !sameVec(x.velocity, y.velocity))) return at + "state";
        if (x.dryMass != y.dryMass || x.fuelMass != y.fuelMass || x.maxThrust != y.maxThrust ||
            x.exhaustVelocity != y.exhaustVelocity || x.cdA != y.cdA || x.radius != y.radius ||
            x.maxTorque != y.maxTorque || x.inertia != y.inertia)
            return at + "parameters";
    }
    return {};
}

void checkRoundTrip(const Scene& original)
{
    std::string text, err;
    REQUIRE_MESSAGE(sceneToJson(original, text, err), err);
    Scene back;
    REQUIRE_MESSAGE(sceneFromJson(text, back, err), err);
    CHECK_MESSAGE(firstDifference(original, back).empty(), "differs at " << firstDifference(original, back));

    // Writing what was read gives the very same text
    std::string again;
    REQUIRE_MESSAGE(sceneToJson(back, again, err), err);
    CHECK(again == text);
}

std::filesystem::path tempFile(const char* name)
{
    return std::filesystem::temp_directory_path() / (std::string("awl_cosmos_test_") + name);
}

} // namespace

TEST_SUITE("scene_json")
{
    TEST_CASE("the plan 11.1 example loads, field for field")
    {
        Scene sc;
        std::string err;
        REQUIRE_MESSAGE(sceneFromJson(kPlanExample, sc, err), err);
        CHECK(sc.name == "Lucerna");
        CHECK(sc.units == "SI");
        CHECK(sc.epoch_s == 0.0);
        REQUIRE(sc.bodies.size() == 2);

        const SceneBody& star = sc.bodies[0];
        CHECK(star.id == 1);
        CHECK(star.kind == BodyKind::Star);
        CHECK(star.kindLocked);
        CHECK(star.fictionalDensity);
        CHECK(star.mu == 1.25e18);
        CHECK(star.radius == 2.4e8);
        CHECK(star.spinPeriod_h == 480.0);
        CHECK(star.colour[1] == 0.95f);
        CHECK_FALSE(star.orbit.has_value());

        const SceneBody& domus = sc.bodies[1];
        CHECK(domus.id == 4);
        REQUIRE(domus.atmosphere.has_value());
        CHECK(domus.atmosphere->scaleHeight == 7000.0);
        REQUIRE(domus.orbit.has_value());
        CHECK(domus.orbit->parent == 1);
        CHECK(domus.orbit->a == 1.45e10);
        CHECK(domus.orbit->e == 0.015);
        CHECK(domus.orbit->argPeriapsis_deg == 103.0);
        CHECK(domus.orbit->meanAnomaly_deg == 35.0);
        CHECK_FALSE(domus.orbit->retrograde);

        REQUIRE(sc.shuttles.size() == 1);
        const SceneShuttle& sh = sc.shuttles[0];
        CHECK(sh.id == 1001);
        CHECK(sh.name == "Shuttle 1");
        CHECK(sh.controller == "Prograde");
        REQUIRE(sh.orbit.has_value());
        CHECK(sh.orbit->parent == 4);
        CHECK(sh.orbit->a == 7.2e5);
        CHECK(sh.dryMass == 2000.0);
        CHECK(sh.fuelMass == 3000.0);
        CHECK(sh.maxThrust == 60000.0);
        CHECK(sh.exhaustVelocity == 3000.0);
        CHECK(sh.cdA == 2.0);
        CHECK(sh.radius == 2.0);
        CHECK(sh.maxTorque == 5000.0);
        CHECK(sh.inertia == 10000.0);
        CHECK_FALSE(sc.settings.infiniteFuel);
        CHECK(sc.settings.controllerTick_s == 0.1);
    }

    TEST_CASE("the plan example resolves: IDs 1 and 4 kept, shuttle placed 720 km from Domus's centre")
    {
        Scene sc;
        std::string err;
        REQUIRE_MESSAGE(sceneFromJson(kPlanExample, sc, err), err);
        ResolvedScene out;
        REQUIRE_MESSAGE(resolveScene(sc, out, err), err);
        CHECK(out.idsPreserved);
        CHECK(out.bodyIds == std::vector<BodyId>{1, 4});
        REQUIRE(out.bodies.size() == 2);
        REQUIRE(out.shuttles.size() == 1);
        const BodyState& s = out.bodies.state;
        const double dx = out.shuttles[0].position.x - s.x[1];
        const double dy = out.shuttles[0].position.y - s.y[1];
        CHECK(std::hypot(dx, dy) == doctest::Approx(7.2e5).epsilon(1e-12));
        const double dvx = out.shuttles[0].velocity.x - s.vx[1];
        const double dvy = out.shuttles[0].velocity.y - s.vy[1];
        CHECK(std::hypot(dvx, dvy) == doctest::Approx(std::sqrt(3.767e12 / 7.2e5)).epsilon(1e-12));
    }

    TEST_CASE("round trip: Lucerna, the plan example, empty, and a hand-built scene with every field")
    {
        Lucerna L;
        checkRoundTrip(L.scene);

        Scene plan;
        std::string err;
        REQUIRE(sceneFromJson(kPlanExample, plan, err));
        checkRoundTrip(plan);

        Scene empty;
        REQUIRE_MESSAGE(loadSceneFile(sceneFile("empty.json"), empty, err), err);
        CHECK(empty.bodies.empty());
        CHECK(empty.shuttles.empty());
        checkRoundTrip(empty);

        // Awkward numbers: every double must come back bit for bit
        Scene odd;
        odd.name = "Odd \"quoted\" name\twith\\escapes \xC3\xA9";
        odd.epoch_s = -123456789.123456789;
        odd.settings.infiniteFuel = true;
        odd.settings.controllerTick_s = 1.0 / 3.0;
        SceneBody a;
        a.id = 9; a.name = "A"; a.kind = BodyKind::NeutronStar; a.kindLocked = true;
        a.mu = 1.0 / 7.0; a.radius = 0.0; a.spinPeriod_h = -1.0e-300; a.inertiaFactor = 0.25;
        a.colour = {0.1f, 1.0f / 3.0f, 0.0f};
        a.atmosphere = Atmosphere{1.0 / 9.0, 123456.789012345};
        a.position = {1.0e-320, -0.0, 6.02214076e23};
        a.velocity = {std::nextafter(1.0, 2.0), 3.0e8, -2.0e-8};
        SceneBody b = a;
        b.id = 2; b.name = "B"; b.kind = BodyKind::BlackHole;
        b.atmosphere.reset();
        b.orbit = SceneOrbit{9, 12345.6789, 0.999999, -720.5, 1e-9, true};
        odd.bodies = {a, b};
        SceneShuttle sh;
        sh.id = 77; sh.name = "S"; sh.controller = "Wander";
        sh.position = {1.5, -2.5, 0.125}; sh.velocity = {1.0e-3, 0.0, 9.0};
        sh.fuelMass = 1.0 / 3.0;
        SceneShuttle sh2 = sh;
        sh2.id = 78; sh2.orbit = SceneOrbit{2, 4.0e6, 0.5, 10.0, 20.0, false};
        odd.shuttles = {sh, sh2};
        checkRoundTrip(odd);
    }

    TEST_CASE("round trip through a file on disk")
    {
        Lucerna L;
        const std::string path = tempFile("roundtrip.json").string();
        std::string err;
        REQUIRE_MESSAGE(saveSceneFile(path, L.scene, err), err);
        Scene back;
        REQUIRE_MESSAGE(loadSceneFile(path, back, err), err);
        CHECK(firstDifference(L.scene, back).empty());
        std::filesystem::remove(path);
    }

    TEST_CASE("saved text is readable: colours as short decimals, one line per orbit")
    {
        Lucerna L;
        std::string text, err;
        REQUIRE(sceneToJson(L.scene, text, err));
        CHECK(text.find("\"colour\": [0.28, 0.48, 0.78]") != std::string::npos);
        CHECK(text.find("\"orbit\": { \"parent\": 4,") != std::string::npos);
        CHECK(text.find("\r") == std::string::npos);
        CHECK(text.back() == '\n');
    }

    TEST_CASE("scenes/lucerna.json agrees with tests/lucerna_data.h (plan section 13)")
    {
        Lucerna L;
        const std::vector<PlacementBody> ref = lucerna_data::lucerna();
        REQUIRE(L.scene.bodies.size() == ref.size());
        for (std::size_t i = 0; i < ref.size(); ++i) {
            const SceneBody& b = L.scene.bodies[i];
            INFO(b.name);
            CHECK(b.id == i + 1);
            CHECK(b.mu == ref[i].mu);
            if (ref[i].parent < 0) {
                CHECK_FALSE(b.orbit.has_value());
                continue;
            }
            REQUIRE(b.orbit.has_value());
            CHECK(b.orbit->parent == static_cast<std::uint64_t>(ref[i].parent) + 1);
            CHECK(b.orbit->a == ref[i].orbit.a);
            CHECK(b.orbit->e == ref[i].orbit.e);
            CHECK(b.orbit->argPeriapsis_deg * lucerna_data::kDegToRad == ref[i].orbit.omega);
            CHECK(b.orbit->meanAnomaly_deg * lucerna_data::kDegToRad == ref[i].orbit.M);
            CHECK(b.orbit->retrograde == ref[i].orbit.retrograde);
        }
    }

    TEST_CASE("defaults: kind Planet, orbit fields, body IDs filled in, shuttle defaults from plan 7.1")
    {
        const std::string text = R"JSON({
            "bodies": [
              { "name": "NoId", "mu": 1e12, "radius": 1000, "position": [0,0,0], "velocity": [0,0,0] },
              { "id": 1, "name": "Has1", "mu": 1e9, "radius": 10,
                "orbit": { "parent": 1, "a": 1e7 } }
            ]})JSON";
        // Has1 orbits itself here, which is invalid: fix the parent after the auto ID is known
        Scene sc;
        std::string err;
        CHECK_FALSE(sceneFromJson(text, sc, err));
        CHECK(err.find("orbits itself") != std::string::npos);

        const std::string ok = R"JSON({
            "bodies": [
              { "name": "NoId", "mu": 1e12, "radius": 1000, "position": [0,0,0], "velocity": [0,0,0] },
              { "id": 1, "name": "Has1", "mu": 1e9, "radius": 10, "orbit": { "parent": 2, "a": 1e7 } },
              { "name": "NoId2", "mu": 1e9, "radius": 10, "position": [5,0,0], "velocity": [0,1,0] }
            ],
            "shuttles": [ { "id": 50, "position": [1,2,3], "velocity": [4,5,6] } ]})JSON";
        REQUIRE_MESSAGE(sceneFromJson(ok, sc, err), err);
        CHECK(sc.bodies[0].id == 2);   // 1 is taken, so the first free ID
        CHECK(sc.bodies[2].id == 3);
        CHECK(sc.bodies[0].kind == BodyKind::Planet);
        CHECK(sc.bodies[0].inertiaFactor == 0.4);
        CHECK(sc.bodies[0].spinPeriod_h == 0.0);
        CHECK_FALSE(sc.bodies[0].atmosphere.has_value());
        CHECK(sc.bodies[1].orbit->e == 0.0);
        CHECK(sc.bodies[1].orbit->argPeriapsis_deg == 0.0);
        CHECK_FALSE(sc.bodies[1].orbit->retrograde);
        CHECK(sc.units == "SI");
        CHECK(sc.settings.controllerTick_s == 0.1);
        CHECK(sc.shuttles[0].controller == "Coast");
        CHECK(sc.shuttles[0].dryMass == 2000.0);
        CHECK(sc.shuttles[0].fuelMass == 3000.0);
        CHECK(sc.shuttles[0].maxThrust == 60000.0);
        CHECK(sc.shuttles[0].exhaustVelocity == 3000.0);
        CHECK(sc.shuttles[0].cdA == 2.0);
        CHECK(sc.shuttles[0].radius == 2.0);
        CHECK(sc.shuttles[0].maxTorque == 5000.0);
        CHECK(sc.shuttles[0].inertia == 10000.0);
        CHECK(sc.shuttles[0].velocity.z == 6.0);
    }

    TEST_CASE("kind names are matched loosely")
    {
        const struct { const char* text; BodyKind kind; } rows[] = {
            {"Gas Giant", BodyKind::GasGiant}, {"gasgiant", BodyKind::GasGiant}, {"DWARF_PLANET", BodyKind::DwarfPlanet},
            {"Neutron-Star", BodyKind::NeutronStar}, {"black hole", BodyKind::BlackHole}, {"Moon", BodyKind::Moon},
            {"Asteroid", BodyKind::Asteroid}, {"star", BodyKind::Star},
        };
        for (const auto& r : rows) {
            const std::string body = std::string(R"({ "id": 1, "kind": ")") + r.text +
                                     R"(", "mu": 1e12, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })";
            Scene sc;
            std::string err;
            INFO(r.text);
            REQUIRE_MESSAGE(sceneFromJson(sceneWith(body), sc, err), err);
            CHECK(sc.bodies[0].kind == r.kind);
        }
    }

    TEST_CASE("malformed input gives an error string, never an exception")
    {
        struct Case { std::string text; const char* expect; };
        const std::vector<Case> cases = {
            {"", "not valid JSON"},
            {"{", "not valid JSON"},
            {"{ \"bodies\": [ }", "not valid JSON"},
            {"not json at all", "not valid JSON"},
            {"[]", "expected an object"},
            {"42", "expected an object"},
            {"null", "expected an object"},
            {"{}", "bodies: required field is missing"},
            {"{ \"bodies\": {} }", "bodies: expected an array"},
            {"{ \"bodies\": [], \"units\": \"cgs\" }", "only \"SI\""},
            {"{ \"bodies\": [], \"epoch_s\": \"soon\" }", "epoch_s: expected a number"},
            {"{ \"bodies\": [], \"shuttles\": 3 }", "shuttles: expected an array"},
            {"{ \"bodies\": [], \"settings\": [] }", "settings: expected an object"},
            {"{ \"bodies\": [], \"settings\": { \"infiniteFuel\": 1 } }", "settings.infiniteFuel: expected true or false"},
            {"{ \"bodies\": [], \"settings\": { \"controllerTick_s\": 0 } }", "controllerTick_s"},
            {sceneWith("7"), "bodies[0]: expected an object"},
            {sceneWith(R"({ "id": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].mu: required field is missing"},
            {sceneWith(R"({ "id": 1, "mu": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].radius: required field is missing"},
            {sceneWith(R"({ "id": 1, "mu": "heavy", "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].mu: expected a number, found string"},
            {sceneWith(R"({ "id": 1, "mu": 0, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "mu must be > 0"},
            {sceneWith(R"({ "id": 1, "mu": -5, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "mu must be > 0"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": -1, "position": [0,0,0], "velocity": [0,0,0] })"), "radius must be >= 0"},
            {sceneWith(R"({ "id": -1, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].id: expected a non-negative integer"},
            {sceneWith(R"({ "id": 1.5, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].id: expected a non-negative integer"},
            {sceneWith(R"({ "id": "one", "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].id: expected a non-negative integer"},
            {sceneWith(R"({ "id": 0, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "id must be nonzero"},
            {sceneWith(std::string(kBodyOk) + "," + kBodyOk), "duplicate id"},
            {sceneWith(R"({ "id": 1, "name": 5, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].name: expected a string"},
            {sceneWith(R"({ "id": 1, "kind": "Comet", "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "unknown kind \"Comet\""},
            {sceneWith(R"({ "id": 1, "kind": 3, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].kind: expected a string"},
            {sceneWith(R"({ "id": 1, "kindLocked": "yes", "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].kindLocked: expected true or false"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "colour": [1, 1], "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].colour: expected an array of 3 numbers"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "colour": [1, 1, "x"], "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].colour[2]: expected a number"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "atmosphere": { "rho0": 1 }, "position": [0,0,0], "velocity": [0,0,0] })"), "bodies[0].atmosphere.scaleHeight: required field is missing"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "atmosphere": { "rho0": 1, "scaleHeight": 0 }, "position": [0,0,0], "velocity": [0,0,0] })"), "scaleHeight > 0"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1 })"), "needs either an \"orbit\" or both"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "position": [0,0,0] })"), "needs either an \"orbit\" or both"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "position": [0,0], "velocity": [0,0,0] })"), "bodies[0].position: expected an array of 3 numbers"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,"fast"] })"), "bodies[0].velocity[2]: expected a number"},
            {sceneWith(R"({ "id": 1, "mu": 1, "radius": 1, "position": [0,0,0], "velocity": [0,0,0], "orbit": { "parent": 1, "a": 1 } })"), "has both an \"orbit\" and"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": 5 })"), "bodies[1].orbit: expected an object"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "a": 1e6 } })"), "bodies[1].orbit.parent: required field is missing"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 1 } })"), "bodies[1].orbit.a: required field is missing"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 1, "a": "far" } })"), "bodies[1].orbit.a: expected a number"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 1, "a": 1e6, "retrograde": "no" } })"), "orbit.retrograde: expected true or false"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 99, "a": 1e6 } })"), "parent id 99 does not exist"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 1, "a": -1e6 } })"), "semi-major axis"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 1, "a": 1e6, "e": 1.5 } })"), "eccentricity"},
            {sceneWith(std::string(kBodyOk) + R"(, { "id": 2, "mu": 1, "radius": 1, "orbit": { "parent": 3, "a": 1e6 } }, { "id": 3, "mu": 1, "radius": 1, "orbit": { "parent": 2, "a": 1e6 } })"), "cycle"},
            {sceneWith(kBodyOk) + "x", "not valid JSON"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"name\": \"no id\", \"position\": [0,0,0], \"velocity\": [0,0,0] } ] }", "shuttles[0].id: required field is missing"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"id\": 5, \"controller\": 7, \"position\": [0,0,0], \"velocity\": [0,0,0] } ] }", "shuttles[0].controller: expected a string"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"id\": 5 } ] }", "shuttles[0]: needs either"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"id\": 1, \"position\": [0,0,0], \"velocity\": [0,0,0] } ] }", "already used"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"id\": 5, \"orbit\": { \"parent\": 8, \"a\": 1e6 } } ] }", "not a body"},
            {"{ \"bodies\": [" + std::string(kBodyOk) + "], \"shuttles\": [ { \"id\": 5, \"dryMass\": -1, \"position\": [0,0,0], \"velocity\": [0,0,0] } ] }", "must be finite and >= 0"},
        };
        for (const Case& c : cases) {
            Scene sc;
            sc.name = "untouched";
            std::string err;
            INFO("input: " << c.text);
            INFO("expecting: " << c.expect);
            bool ok = true;
            CHECK_NOTHROW(ok = sceneFromJson(c.text, sc, err));
            CHECK_FALSE(ok);
            CHECK(err.find(c.expect) != std::string::npos);
            CHECK_MESSAGE(sc.name == "untouched", "a failed load must not modify the output scene");
        }
    }

    TEST_CASE("number out of range and invalid UTF-8 are errors too")
    {
        Scene sc;
        std::string err;
        CHECK_FALSE(sceneFromJson(sceneWith(R"({ "id": 1, "mu": 1e999, "radius": 1, "position": [0,0,0], "velocity": [0,0,0] })"), sc, err));
        CHECK_FALSE(err.empty());

        Scene bad;
        SceneBody b;
        b.id = 1; b.mu = 1.0; b.name = "bad \xFF\xFE name";
        bad.bodies = {b};
        std::string text;
        CHECK_FALSE(sceneToJson(bad, text, err));
        CHECK(err.find("could not be written") != std::string::npos);
    }

    TEST_CASE("saving refuses an invalid scene; files that cannot be read or written are errors")
    {
        Scene sc;
        SceneBody b;
        b.id = 1; b.mu = std::nan("");
        sc.bodies = {b};
        std::string text, err;
        CHECK_FALSE(sceneToJson(sc, text, err));
        CHECK(err.find("mu must be > 0") != std::string::npos);

        Scene loaded;
        CHECK_FALSE(loadSceneFile(sceneFile("does_not_exist.json"), loaded, err));
        CHECK(err.find("cannot open scene file") != std::string::npos);

        const std::string badPath = (std::filesystem::temp_directory_path() / "awl_cosmos_no_such_dir" / "x.json").string();
        CHECK_FALSE(saveSceneFile(badPath, Scene{}, err));
        CHECK(err.find("cannot open") != std::string::npos);

        // A file with broken content names the file
        const std::string path = tempFile("broken.json").string();
        {
            std::ofstream f(path, std::ios::binary);
            f << "{ \"bodies\": [ { \"id\": 1 } ] }";
        }
        CHECK_FALSE(loadSceneFile(path, loaded, err));
        CHECK(err.find("awl_cosmos_test_broken.json") != std::string::npos);
        CHECK(err.find("bodies[0].mu: required field is missing") != std::string::npos);
        std::filesystem::remove(path);
    }
}
