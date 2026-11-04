#pragma once
#include <doctest.h>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include "app/editor_model.h"
#include "core/scene.h"
#include "test_scene_util.h"

// Helpers shared by the editor, run-control and preset tests (M6).
namespace editor_test {

using namespace cosmos;
using namespace cosmos::app;

inline bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
inline bool sameBits(const Vec3& a, const Vec3& b) { return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z); }

inline bool sameOrbit(const std::optional<SceneOrbit>& a, const std::optional<SceneOrbit>& b)
{
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    return a->parent == b->parent && sameBits(a->a, b->a) && sameBits(a->e, b->e) && sameBits(a->argPeriapsis_deg, b->argPeriapsis_deg) &&
           sameBits(a->meanAnomaly_deg, b->meanAnomaly_deg) && a->retrograde == b->retrograde;
}

// Every field equal, doubles bit for bit
inline bool sameScene(const Scene& a, const Scene& b)
{
    if (a.name != b.name || a.units != b.units || !sameBits(a.epoch_s, b.epoch_s) || a.bodies.size() != b.bodies.size() ||
        a.shuttles.size() != b.shuttles.size() || a.settings.infiniteFuel != b.settings.infiniteFuel ||
        !sameBits(a.settings.controllerTick_s, b.settings.controllerTick_s))
        return false;
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const SceneBody &x = a.bodies[i], &y = b.bodies[i];
        if (x.id != y.id || x.name != y.name || x.kind != y.kind || x.kindLocked != y.kindLocked ||
            x.fictionalDensity != y.fictionalDensity || !sameBits(x.mu, y.mu) || !sameBits(x.radius, y.radius) ||
            !sameBits(x.spinPeriod_h, y.spinPeriod_h) || !sameBits(x.inertiaFactor, y.inertiaFactor) || x.colour != y.colour ||
            x.atmosphere.has_value() != y.atmosphere.has_value() || !sameOrbit(x.orbit, y.orbit) || !sameBits(x.position, y.position) ||
            !sameBits(x.velocity, y.velocity))
            return false;
        if (x.atmosphere && (!sameBits(x.atmosphere->rho0, y.atmosphere->rho0) || !sameBits(x.atmosphere->scaleHeight, y.atmosphere->scaleHeight)))
            return false;
    }
    for (std::size_t i = 0; i < a.shuttles.size(); ++i) {
        const SceneShuttle &x = a.shuttles[i], &y = b.shuttles[i];
        if (x.id != y.id || x.name != y.name || x.controller != y.controller || !sameOrbit(x.orbit, y.orbit) ||
            !sameBits(x.position, y.position) || !sameBits(x.velocity, y.velocity) || !sameBits(x.dryMass, y.dryMass) ||
            !sameBits(x.fuelMass, y.fuelMass) || !sameBits(x.maxThrust, y.maxThrust) || !sameBits(x.exhaustVelocity, y.exhaustVelocity) ||
            !sameBits(x.cdA, y.cdA) || !sameBits(x.radius, y.radius) || !sameBits(x.maxTorque, y.maxTorque) || !sameBits(x.inertia, y.inertia))
            return false;
    }
    return true;
}

// scenes/lucerna.json without its demo shuttles
inline Scene lucerna()
{
    scene_test::Lucerna L;
    return L.scene;
}

inline std::uint64_t idOf(const Scene& s, const std::string& name)
{
    for (const SceneBody& b : s.bodies)
        if (b.name == name) return b.id;
    FAIL("no body named " << name);
    return 0;
}

inline bool anyContains(const std::vector<std::string>& v, const char* part)
{
    for (const std::string& s : v)
        if (s.find(part) != std::string::npos) return true;
    return false;
}

} // namespace editor_test
