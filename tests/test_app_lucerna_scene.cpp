#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include "core/events.h"
#include "core/scene_json.h"
#include "core/simulation.h"
#include "test_scene_util.h"

using namespace cosmos;

// M4 integration: scenes/lucerna.json ships a handful of demo shuttles (plan 11.1 example) that the app shows. They must
// load, resolve and fly: Coast shuttles keep their orbit, the Retrograde one is the one that falls into Domus's air.
TEST_SUITE("app_lucerna_scene")
{
    TEST_CASE("the shipped Lucerna scene: 6-10 shuttles around Domus, Custos and Gelu with mixed controllers, all outside the air")
    {
        Scene scene;
        std::string err;
        REQUIRE_MESSAGE(loadSceneFile(scene_test::sceneFile("lucerna.json"), scene, err), err);
        REQUIRE(scene.shuttles.size() >= 6);
        REQUIRE(scene.shuttles.size() <= 10);

        std::set<std::string> controllers;
        std::set<std::uint64_t> ids;
        for (const SceneShuttle& sh : scene.shuttles) {
            INFO(sh.name);
            CHECK(ids.insert(sh.id).second);
            CHECK(findSceneBody(scene, sh.id) < 0);   // bodies and shuttles share one ID space
            controllers.insert(sh.controller);
            REQUIRE(sh.orbit.has_value());
            const int pi = findSceneBody(scene, sh.orbit->parent);
            REQUIRE(pi >= 0);
            const SceneBody& parent = scene.bodies[static_cast<std::size_t>(pi)];
            CHECK((parent.name == "Domus" || parent.name == "Custos" || parent.name == "Gelu"));
            // Clear of the surface: 100 km above it where there is air (Domus's air tops out near 98 km, plan 13), so the
            // Coast shuttles stay out of it and the burning ones start outside it; 20 % of the radius for the airless moons
            const double periapsis = sh.orbit->a * (1.0 - sh.orbit->e);
            CHECK(periapsis >= (parent.atmosphere ? parent.radius + 1.0e5 : 1.2 * parent.radius));
        }
        CHECK(controllers.size() >= 4);   // Coast, Prograde, Retrograde and Wander all appear
    }

    TEST_CASE("the shipped shuttles fly a day: Coast orbits keep their radius, the Retrograde one is destroyed by Domus")
    {
        Scene scene;
        std::string err;
        REQUIRE_MESSAGE(loadSceneFile(scene_test::sceneFile("lucerna.json"), scene, err), err);
        ResolvedScene res;
        REQUIRE_MESSAGE(resolveScene(scene, res, err), err);
        CHECK(res.warnings.empty());

        SimulationOptions opt;
        opt.threads = 4;
        Simulation sim(res, scene.settings, opt);
        REQUIRE(sim.warnings().empty());
        REQUIRE(sim.shuttles().size() == scene.shuttles.size());
        REQUIRE(sim.shuttles().aliveCount() == scene.shuttles.size());

        // Distance to the scene parent, sampled once an hour for the day. A Coast shuttle on a circular orbit stays near its
        // radius a: the tides of the star (and, for the Custos and Gelu shuttles, of Domus) bend it and the air at 100 km
        // drags it, but none of that moves it far in a day. Measured worst |r/a - 1| (release and debug): 1e-4 for the
        // 100 km Domus shuttles, 5e-3 at synchronous height, 2e-4 around Gelu and 2.7e-2 around Custos (0.24 of its Hill
        // radius, the loosest orbit); the bound below leaves a factor of 1.8 over the largest.
        auto distance = [&](std::size_t i, const SceneShuttle& sc) {
            const std::size_t b = *sim.bodies().find(res.bodyIds[static_cast<std::size_t>(findSceneBody(scene, sc.orbit->parent))]);
            const Shuttles& s = sim.shuttles();
            return std::hypot(s.state.x[i] - sim.bodies().state.x[b], s.state.y[i] - sim.bodies().state.y[b]);
        };
        std::vector<double> worst(scene.shuttles.size(), 0.0);
        std::vector<Event> events;
        for (int hour = 1; hour <= 24; ++hour) {
            REQUIRE(sim.advanceTo(3600.0 * hour));
            const std::vector<Event> got = sim.drainEvents();
            events.insert(events.end(), got.begin(), got.end());
            for (std::size_t i = 0; i < scene.shuttles.size(); ++i)
                if (sim.shuttles().alive[i]) worst[i] = std::max(worst[i], std::abs(distance(i, scene.shuttles[i]) / scene.shuttles[i].orbit->a - 1.0));
        }

        double worstCoast = 0.0;
        for (std::size_t i = 0; i < scene.shuttles.size(); ++i) {
            const SceneShuttle& sc = scene.shuttles[i];
            INFO(sc.name);
            if (sc.controller == "Coast") {
                CHECK(sim.shuttles().alive[i] == 1);
                worstCoast = std::max(worstCoast, worst[i]);
                CHECK(worst[i] < 0.05);
            }
            if (sc.controller == "Retrograde") {
                CHECK(sim.shuttles().alive[i] == 0);
                bool destroyedByDomus = false;
                for (const Event& e : events)
                    if (e.type == EventType::ShuttleDestroyed && e.id == sc.id && e.other == res.bodyIds[static_cast<std::size_t>(findSceneBody(scene, 4))])
                        destroyedByDomus = true;
                CHECK(destroyedByDomus);
            }
        }
        MESSAGE("worst Coast |r/a - 1| over one day: " << worstCoast);
    }
}
