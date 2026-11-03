#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "core/bodies.h"
#include "core/vec3.h"

namespace cosmos {

// Scene model and resolver (plan sections 11.1, 5.9). A Scene is the file-level description: bodies and shuttles
// given either by orbital elements or by explicit state. resolveScene turns it into a Bodies store plus shuttle
// start states. No OpenGL, no JSON here (JSON lives in scene_json.h).

// Orbit about a parent body (a scene body id). Angles in degrees as in the file; bound orbits only (0 <= e < 1, a > 0).
struct SceneOrbit {
    std::uint64_t parent = 0;
    double a = 0.0;
    double e = 0.0;
    double argPeriapsis_deg = 0.0;   // counter-clockwise from +x
    double meanAnomaly_deg = 0.0;    // at the scene epoch
    bool retrograde = false;         // clockwise
};

struct SceneBody {
    std::uint64_t id = 0;            // permanent scene ID, unique and nonzero
    std::string name;
    BodyKind kind = BodyKind::Planet;
    bool kindLocked = false;
    bool fictionalDensity = false;
    double mu = 0.0;                 // GM, m^3/s^2
    double radius = 0.0;             // m
    // Signed: positive spins counter-clockwise (+z), negative is a retrograde (clockwise) spin, 0 means no spin.
    double spinPeriod_h = 0.0;
    double inertiaFactor = 0.4;
    std::optional<Atmosphere> atmosphere;
    std::array<float, 3> colour = {1.0f, 1.0f, 1.0f};
    // Either an orbit, or an explicit state (position/velocity; zero by default). Setting an orbit makes the state ignored.
    std::optional<SceneOrbit> orbit;
    Vec3 position;
    Vec3 velocity;
};

// Plain data for M3 (plan sections 7.1, 11.1). Defaults are the 7.1 table.
struct SceneShuttle {
    std::uint64_t id = 0;
    std::string name;
    std::string controller = "Coast";
    std::optional<SceneOrbit> orbit;   // parent is a body id; the shuttle is massless
    Vec3 position;
    Vec3 velocity;
    double dryMass = 2000.0;
    double fuelMass = 3000.0;
    double maxThrust = 60000.0;
    double exhaustVelocity = 3000.0;
    double cdA = 2.0;
    double radius = 2.0;
    double maxTorque = 5000.0;
    double inertia = 1.0e4;
};

struct SceneSettings {
    bool infiniteFuel = false;
    double controllerTick_s = 0.1;
};

struct Scene {
    std::string name;
    std::string units = "SI";
    double epoch_s = 0.0;
    std::vector<SceneBody> bodies;
    std::vector<SceneShuttle> shuttles;
    SceneSettings settings;
};

// Whether resolveScene shifts the result to the system barycentre with zero total momentum (plan 5.9 step 4).
// Auto: only when at least one body is placed by an orbit. A scene made only of explicit states (for example Horizons
// vectors) is then taken verbatim. Always / Never force the choice.
enum class Recentre { Auto, Always, Never };

struct ResolveOptions {
    Recentre recentre = Recentre::Auto;
};

struct ResolvedShuttle {
    SceneShuttle desc;   // as in the scene
    Vec3 position;       // resolved, in the same frame as the bodies
    Vec3 velocity;
};

struct ResolvedScene {
    Bodies bodies;
    // bodyIds[i] is the Bodies ID of scene.bodies[i]. See resolveScene for when it equals the scene ID.
    std::vector<BodyId> bodyIds;
    bool idsPreserved = true;
    std::vector<ResolvedShuttle> shuttles;
    std::vector<std::string> warnings;
};

// Checks a scene without resolving it. Returns true when it is valid, otherwise false with a message that names the
// offending body or shuttle. Never throws.
bool validateScene(const Scene& scene, std::string& error);

// Plan 5.9 / 11.1. Each body without an orbit is a root and stays at its given state; the bodies orbiting it (directly
// or through other bodies) are placed in Jacobi coordinates with placeJacobi. Then, depending on opts.recentre, the whole
// scene is moved to the barycentre with zero total momentum.
//
// Body order and IDs: bodies are added to the store in ascending scene ID, so the store order is the ID order, not the
// file order. A Bodies ID equals the scene ID whenever the store can reach it (store IDs only count up; gaps up to
// 100000 are skipped by adding and removing placeholder bodies). A body whose ID is out of reach gets the next free
// store ID instead, idsPreserved becomes false and a warning is added. Always go through bodyIds for the mapping.
//
// Shuttles: an orbit is converted with the Kepler utilities against the parent body's resolved state (the shuttle is
// massless, so mu = mu_parent). Returns false with an error string for an invalid scene; never throws.
bool resolveScene(const Scene& scene, ResolvedScene& out, std::string& error, const ResolveOptions& opts = {});

// Index of the scene body with this scene ID, or -1.
int findSceneBody(const Scene& scene, std::uint64_t id);

} // namespace cosmos
