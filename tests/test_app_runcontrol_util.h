#pragma once
#include <doctest.h>
#include "app/run_control.h"
#include "test_app_driver_util.h"
#include "test_app_editor_util.h"

// Helpers for the run-control tests (M6).
namespace rc_test {

using namespace cosmos;
using namespace cosmos::app;
using namespace editor_test;

// Repeatable runs: the physics may take as long as it needs, so the result never depends on wall time
inline RunOptions options()
{
    RunOptions o;
    o.driver.physicsBudget_s = 0.0;
    o.sim.threads = 1;
    return o;
}

// Lucerna (no demo shuttles) plus n shuttles in orbits of different sizes about Domus, IDs 900...
inline Scene lucernaWithShuttles(int n)
{
    Scene s = lucerna();
    const std::uint64_t domus = idOf(s, "Domus");
    const char* controllers[3] = {"Coast", "Prograde", "Wander"};
    for (int k = 0; k < n; ++k) {
        SceneShuttle sh;
        sh.id = 900 + static_cast<std::uint64_t>(k);
        sh.name = "Shuttle " + std::to_string(k + 1);
        sh.controller = controllers[k % 3];
        SceneOrbit o;
        o.parent = domus;
        o.a = 8.2e5 + 1.5e5 * k;
        o.meanAnomaly_deg = 40.0 * k;
        sh.orbit = o;
        s.shuttles.push_back(sh);
    }
    return s;
}

// Two bodies on a head-on course that merge after about 40 s, a heavy bystander C with a shuttle that falls into it and is
// destroyed, and a second shuttle far away that burns prograde (and so loses fuel). Explicit states only. C spins (5 h).
inline Scene collisionScene()
{
    Scene s;
    s.name = "Collision";
    const auto body = [](std::uint64_t id, const char* name, double mu, double radius, Vec3 r, Vec3 v) {
        SceneBody b;
        b.id = id;
        b.name = name;
        b.mu = mu;
        b.radius = radius;
        b.position = r;
        b.velocity = v;
        return b;
    };
    s.bodies.push_back(body(1, "A", 1.0e10, 1.0e3, {-5.0e3, 0.0, 0.0}, {100.0, 0.0, 0.0}));
    s.bodies.push_back(body(2, "B", 1.0e10, 1.0e3, {5.0e3, 0.0, 0.0}, {-100.0, 0.0, 0.0}));
    s.bodies.push_back(body(3, "C", 1.0e12, 1.0e4, {0.0, 1.0e6, 0.0}, {}));
    s.bodies[2].spinPeriod_h = 5.0;
    s.bodies[2].atmosphere = Atmosphere{1.0, 5000.0};
    s.bodies[2].colour = {0.2f, 0.4f, 0.6f};
    SceneShuttle doomed;
    doomed.id = 10;
    doomed.name = "Doomed";
    doomed.position = {0.0, 1.05e6, 0.0};
    s.shuttles.push_back(doomed);
    SceneShuttle survivor;
    survivor.id = 11;
    survivor.name = "Survivor";
    survivor.controller = "Prograde";
    survivor.position = {3.0e6, 3.0e6, 0.0};
    survivor.velocity = {0.0, -20.0, 0.0};
    s.shuttles.push_back(survivor);
    return s;
}

inline void runFrames(RunControl& rc, int frames, double dt = 0.1)
{
    for (int i = 0; i < frames; ++i) rc.advance(dt);
}

// Runs until the physics has produced a merge and a shuttle loss (or gives up)
inline bool runUntilMergeAndLoss(RunControl& rc, int maxFrames = 300)
{
    for (int i = 0; i < maxFrames; ++i) {
        rc.advance(0.25);
        const SimulationStats& st = rc.driver().sim().stats();
        if (st.merges >= 1 && st.shuttlesDestroyed >= 1) return true;
    }
    return false;
}

} // namespace rc_test
