#pragma once
#include <doctest.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include "app/sim_driver.h"
#include "test_simulation_util.h"

// Helpers shared by the SimDriver tests (M4).
namespace driver_test {

using namespace cosmos;
using namespace cosmos::app;

// Lucerna (scenes/lucerna.json) with `shuttles` extra shuttles in circular orbits of different radii around Domus, with
// different autopilots so that they thrust and turn (heading and throttle change). IDs 900, 901, ... never collide with
// body IDs (1..14).
inline ResolvedScene lucernaWithShuttles(int shuttles)
{
    scene_test::Lucerna L;
    const std::size_t domus = L.storeIndex("Domus");
    ResolvedScene res = std::move(L.res);
    const double mu = res.bodies.mu[domus];
    const double R = res.bodies.radius[domus];
    const char* controllers[3] = {"Coast", "Prograde", "Wander"};
    for (int k = 0; k < shuttles; ++k) {
        Vec3 pos, vel;
        sim_test::circularOrbit(mu, R + 2.0e5 + 1.5e5 * k, 0.7 + 1.9 * k, sim_test::bodyPos(res.bodies, domus),
                                sim_test::bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(sim_test::resolvedShuttle(900 + static_cast<std::uint64_t>(k), pos, vel, controllers[k % 3]));
    }
    return res;
}

inline bool sameBits(const Vec3& a, const Vec3& b)
{
    return sim_test::sameBits(a.x, b.x) && sim_test::sameBits(a.y, b.y) && sim_test::sameBits(a.z, b.z);
}

// Bit-for-bit equality of two display states (every number the renderer would read).
inline bool sameBits(const DisplayState& a, const DisplayState& b)
{
    if (!sim_test::sameBits(a.t, b.t) || a.bodies.size() != b.bodies.size() || a.shuttles.size() != b.shuttles.size()) return false;
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const DisplayBody &x = a.bodies[i], &y = b.bodies[i];
        if (x.id != y.id || x.kind != y.kind || x.name != y.name || x.interpolated != y.interpolated || x.absorbed != y.absorbed ||
            !sim_test::sameBits(x.mu, y.mu) || !sim_test::sameBits(x.radius, y.radius) || !sim_test::sameBits(x.spinRate, y.spinRate) ||
            !sameBits(x.r, y.r) || !sameBits(x.v, y.v))
            return false;
    }
    for (std::size_t i = 0; i < a.shuttles.size(); ++i) {
        const DisplayShuttle &x = a.shuttles[i], &y = b.shuttles[i];
        if (x.id != y.id || x.index != y.index || x.interpolated != y.interpolated || x.dying != y.dying || !sameBits(x.r, y.r) ||
            !sameBits(x.v, y.v) || !sim_test::sameBits(x.heading, y.heading) || !sim_test::sameBits(x.spin, y.spin) ||
            !sim_test::sameBits(x.throttle, y.throttle))
            return false;
    }
    return true;
}

inline const DisplayBody* findBody(const DisplayState& s, BodyId id)
{
    for (const DisplayBody& b : s.bodies)
        if (b.id == id && !b.absorbed) return &b;
    return nullptr;
}

inline const DisplayShuttle* findShuttle(const DisplayState& s, ShuttleId id)
{
    for (const DisplayShuttle& b : s.shuttles)
        if (b.id == id) return &b;
    return nullptr;
}

// The smallest time strictly after t: asks the driver for exactly one more body step.
inline double justAfter(double t) { return std::nextafter(t, 1.0e300); }

} // namespace driver_test
