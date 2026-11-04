#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <string>
#include "core/hierarchy.h"
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// What a controller sees as its parent (plan 5.9, 15 item 2c), the initial heading of a scene shuttle (7.1), and the
// events of a shuttle that dies in the step in which its tank runs dry (5.7, 5.11).

namespace {

// Keeps the first observation of every shuttle (by ID) and never burns or asks again.
struct ObservationRecorder final : IController {
    ShuttleCommand update(const ShuttleObservation& o, Rng&) override
    {
        if (first.find(o.shuttleId) == first.end()) first[o.shuttleId] = o;
        return {};
    }
    std::int64_t nextTick(const ShuttleObservation&) const override { return std::numeric_limits<std::int64_t>::max(); }
    std::map<std::uint64_t, ShuttleObservation> first;
};

struct FullThrottle final : IController {
    ShuttleCommand update(const ShuttleObservation&, Rng&) override { return {1.0, 0.0}; }
    std::int64_t nextTick(const ShuttleObservation&) const override { return std::numeric_limits<std::int64_t>::max(); }
};

} // namespace

TEST_SUITE("simulation parents")
{
    TEST_CASE("the parent a controller sees is the body with the smallest Hill sphere around the shuttle (Lucerna moons)")
    {
        scene_test::Lucerna L;
        ResolvedScene res = L.res;
        const Bodies& B = res.bodies;
        std::vector<Vec3> r(B.size()), v(B.size());
        for (std::size_t i = 0; i < B.size(); ++i) {
            r[i] = bodyPos(B, i);
            v[i] = bodyVel(B, i);
        }
        const Hierarchy h = buildHierarchy(r, v, B.mu);

        struct Spot {
            std::size_t body;
            double dist;
            std::uint64_t id;
        };
        std::vector<Spot> spots;
        std::uint64_t id = 5000;
        for (const char* name : {"Domus", "Gelu", "Custos", "Exul"}) {
            const std::size_t b = L.storeIndex(name);
            // Three radii inside the body's own sphere: fixed ones for the planet (its sphere is the system's, 1.4e8 m, and
            // its moons orbit at 7.6e6 m and beyond), fractions of the Hill radius for the moons
            std::vector<double> radii;
            if (std::string(name) == "Domus") {
                radii = {1.0e6, 2.0e6, 3.0e6};
            } else {
                const double hill = hillRadius(h, static_cast<int>(b));
                REQUIRE(std::isfinite(hill));
                REQUIRE(hill > 4.0 * B.radius[b]);
                radii = {std::max(1.5 * B.radius[b], 0.15 * hill), 0.35 * hill, 0.65 * hill};
            }
            for (double d : radii) spots.push_back({b, d, id++});
        }
        for (const Spot& sp : spots) {
            // The direction (of 16) that stays farthest from every other body
            Vec3 best;
            double bestClear = -1.0;
            for (int k = 0; k < 16; ++k) {
                const double a = 2.0 * kPi * k / 16.0;
                const Vec3 p = r[sp.body] + Vec3{std::cos(a), std::sin(a), 0.0} * sp.dist;
                double clear = std::numeric_limits<double>::infinity();
                for (std::size_t j = 0; j < B.size(); ++j)
                    if (j != sp.body) clear = std::min(clear, norm(p - r[j]));
                if (clear > bestClear) {
                    bestClear = clear;
                    best = p;
                }
            }
            res.shuttles.push_back(resolvedShuttle(sp.id, best, v[sp.body], "Prograde"));
        }

        ObservationRecorder rec;
        Simulation sim(res, SceneSettings{});
        sim.controllers().set(ControllerKind::Prograde, &rec);
        REQUIRE(sim.shuttles().size() == spots.size());
        REQUIRE(sim.advanceTo(1.0));
        for (const Spot& sp : spots) {
            INFO("body " << B.name[sp.body] << ", distance " << sp.dist);
            const auto it = rec.first.find(sp.id);
            REQUIRE(it != rec.first.end());
            const ShuttleObservation& o = it->second;
            CHECK(o.parentId == B.ids[sp.body]);
            CHECK(o.parentMu == B.mu[sp.body]);
            CHECK(std::fabs(norm(o.relPosition) - sp.dist) < 1.0e-3);   // the parent is at the body: exact at the start
            CHECK(norm(o.relVelocity) < 1.0e-6);                         // the shuttle moves with the body
        }
    }

    TEST_CASE("a scene shuttle starts pointing along its velocity relative to its parent, not along the absolute velocity")
    {
        scene_test::Lucerna L;
        ResolvedScene res = L.res;
        const std::size_t domus = L.storeIndex("Domus");
        const Vec3 vD = bodyVel(res.bodies, domus);
        // Circular counter-clockwise orbit with the phase chosen so that the orbital velocity is at right angles to the
        // motion of the planet about the star: the absolute velocity then points almost along the planet's own.
        const double phase = std::atan2(vD.y, vD.x);
        Vec3 pos, vel;
        circularOrbit(res.bodies.mu[domus], res.bodies.radius[domus] + 3.0e6, phase, bodyPos(res.bodies, domus), vD, pos, vel);
        res.shuttles.push_back(resolvedShuttle(777, pos, vel, "Coast"));
        Simulation sim(res, SceneSettings{});
        REQUIRE(sim.shuttles().size() == 1);
        const Vec3 rel = vel - vD;
        const double expected = std::atan2(rel.y, rel.x);   // phase + 90 degrees
        const double absolute = std::atan2(vel.y, vel.x);
        CHECK(sim.shuttles().heading[0] == doctest::Approx(expected).epsilon(1e-12));
        CHECK(std::fabs(wrapAngle(expected - absolute)) > 1.0);   // the old heading was more than a radian away
    }

    TEST_CASE("fuel running out in the step in which the shuttle hits the surface: no FuelEmpty after the destruction")
    {
        // A planet (mu 4e12, R 1e6, g = 4 m/s2 at the surface) with a shuttle 100 m above it, falling at 10 m/s with the
        // engine pushing it down (30 m/s2): it lands after about 1.9 s. The tank (48 kg at 20 kg/s) is empty after 2.4 s,
        // which is the end of that step (the fuel-out time ends a step), so the touchdown is inside it.
        FullThrottle thrust;
        ResolvedScene res = sceneOf({bodyDesc(4.0e12, 1.0e6, {}, {})});
        SimulationOptions opt;
        opt.ias15.dtMax = 10.0;
        Simulation sim(res, SceneSettings{}, opt);
        sim.controllers().set(ControllerKind::Prograde, &thrust);
        ShuttleDesc d;
        d.id = 31;
        d.controller = ControllerKind::Prograde;
        d.position = {1.0e6 + 102.0, 0.0, 0.0};   // 100 m above the surface, for the 2 m shuttle radius
        d.velocity = {-10.0, 0.0, 0.0};
        d.heading = kPi;                          // thrust towards the planet
        d.dryMass = 2000.0;
        d.fuelMass = 48.0;
        REQUIRE(sim.addShuttle(d) == 31);
        REQUIRE(sim.advanceTo(10.0));
        const std::vector<Event> ev = sim.drainEvents();
        const Event* dead = firstOf(ev, EventType::ShuttleDestroyed, 31);
        REQUIRE(dead != nullptr);
        std::printf("FUEL+HIT destroyed at %.4f s, FuelEmpty events %zu (tank empty at 2.4 s)\n", dead->t,
                    countOf(ev, EventType::FuelEmpty));
        CHECK(dead->t > 1.5);
        CHECK(dead->t < 2.3);
        for (const Event& e : ev)
            if (e.id == 31) CHECK(e.t <= dead->t);   // nothing of the shuttle happens after it is gone
        CHECK(countOf(ev, EventType::FuelEmpty) == 0);
    }
}
