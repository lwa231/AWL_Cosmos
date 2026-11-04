#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "core/atmosphere.h"
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// Shuttles inside the runner: atmosphere entry and destruction (plan 5.5, 8 items 1-3, 4.7), the swept shuttle-vs-body
// test at very large body steps (5.11, 14 "Swept collision"), conservation over a Lucerna run (14, 15 M3 done-when).

TEST_SUITE("simulation shuttles")
{
    TEST_CASE("a Retrograde shuttle in low Domus orbit raises AtmosphereEntry, then is destroyed at the surface")
    {
        scene_test::Lucerna L;
        const std::size_t domus = L.storeIndex("Domus");
        ResolvedScene res = std::move(L.res);
        const double muD = res.bodies.mu[domus];
        const double RD = res.bodies.radius[domus];
        const double top = atmosphereTop(*res.bodies.atmosphere[domus]);   // about 98 km
        REQUIRE(top > 5.0e4);

        // 150 km above the surface: outside the atmosphere (top 98 km), counter-clockwise circular orbit
        Vec3 pos, vel;
        circularOrbit(muD, RD + 1.5e5, 1.0, bodyPos(res.bodies, domus), bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(resolvedShuttle(900, pos, vel, "Retrograde"));
        const BodyId domusId = res.bodies.ids[domus];

        SimulationOptions opt;
        opt.ias15.dtMax = 120.0;   // short body steps: the whole descent spans many of them
        Simulation sim(res, SceneSettings{}, opt);
        REQUIRE(sim.shuttles().size() == 1);

        std::vector<Event> all;
        Vec3 domusAtDeath;
        bool haveDeath = false;
        double deathT = 0.0;
        while (sim.shuttles().alive[0] != 0 && sim.time() < 6000.0) {
            REQUIRE(sim.advanceTo(sim.time() + 1.0));   // exactly one body step per call
            const std::vector<Event> ev = sim.drainEvents();
            for (const Event& e : ev) {
                all.push_back(e);
                if (e.type == EventType::ShuttleDestroyed) {
                    // The dense output of the step that contains the event is still the latest one
                    REQUIRE(e.t >= sim.bodyStepStart());
                    REQUIRE(e.t <= sim.bodyStepEnd());
                    REQUIRE(sim.bodyPositionAt(domusId, e.t, domusAtDeath));
                    deathT = e.t;
                    haveDeath = true;
                }
            }
        }
        REQUIRE(sim.shuttles().alive[0] == 0);
        REQUIRE(haveDeath);

        // Exactly one entry and one destruction, the entry first, both about Domus
        CHECK(countOf(all, EventType::AtmosphereEntry) == 1);
        CHECK(countOf(all, EventType::ShuttleDestroyed) == 1);
        const Event* entry = firstOf(all, EventType::AtmosphereEntry);
        const Event* dead = firstOf(all, EventType::ShuttleDestroyed);
        REQUIRE(entry != nullptr);
        REQUIRE(dead != nullptr);
        CHECK(entry->id == 900);
        CHECK(entry->other == domusId);
        CHECK(dead->id == 900);
        CHECK(dead->other == domusId);
        CHECK(entry->t < dead->t);
        CHECK(entry->t > 0.0);
        const auto posE = std::find_if(all.begin(), all.end(), [](const Event& e) { return e.type == EventType::AtmosphereEntry; });
        const auto posD = std::find_if(all.begin(), all.end(), [](const Event& e) { return e.type == EventType::ShuttleDestroyed; });
        CHECK(posE < posD);   // delivered in time order
        CHECK(sim.stats().shuttlesDestroyed == 1);

        // Destroyed at the surface: the shuttle's final state is the contact point, at R + shuttle radius from Domus
        const double contactR = RD + sim.shuttles().radius[0];
        const double miss = std::fabs(norm(shuttlePos(sim.shuttles(), 0) - domusAtDeath) - contactR);
        std::printf("DESTROY entry t = %.1f s, destroyed t = %.1f s, radial miss %.3e m\n", entry->t, deathT, miss);
        CHECK(miss < 0.01);   // measured 1e-6 m: the contact point is on the chord model of the last shuttle step, 0.1 s long
        // and it stays there: later steps leave a dead shuttle alone
        const Vec3 where = shuttlePos(sim.shuttles(), 0);
        REQUIRE(sim.advanceTo(sim.time() + 500.0));
        CHECK(norm(shuttlePos(sim.shuttles(), 0) - where) == 0.0);
        CHECK(sim.drainEvents().empty());
        CHECK(sim.shuttles().alive[0] == 0);
        CHECK(sim.shuttleStep(0).to.t == deathT);
    }

    TEST_CASE("a shuttle that starts inside the atmosphere raises no AtmosphereEntry, only the destruction")
    {
        scene_test::Lucerna L;
        const std::size_t domus = L.storeIndex("Domus");
        ResolvedScene res = std::move(L.res);
        const double RD = res.bodies.radius[domus];
        Vec3 pos, vel;
        circularOrbit(res.bodies.mu[domus], RD + 5.0e4, 1.0, bodyPos(res.bodies, domus), bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(resolvedShuttle(901, pos, vel, "Retrograde"));
        SimulationOptions opt;
        opt.ias15.dtMax = 120.0;
        Simulation sim(res, SceneSettings{}, opt);
        std::vector<Event> all;
        while (sim.shuttles().alive[0] != 0 && sim.time() < 6000.0) {
            REQUIRE(sim.advanceTo(sim.time() + 1.0));
            for (const Event& e : sim.drainEvents()) all.push_back(e);
        }
        CHECK(sim.shuttles().alive[0] == 0);
        CHECK(countOf(all, EventType::AtmosphereEntry) == 0);
        CHECK(countOf(all, EventType::ShuttleDestroyed) == 1);
    }

    TEST_CASE("the swept test catches a fast shuttle when the body step is very large")
    {
        // A heavy star far away keeps the IAS15 step long; the planet (R = 6.4e6 m) is hit head-on by a shuttle at 1e5 m/s
        // that starts 1e9 m behind it. Both bodies move; the planet's orbit speed is 11.4 km/s.
        const double muStar = 1.3e20, rOrbit = 1.0e12;
        const double vOrbit = std::sqrt(muStar / rOrbit);
        BodyDesc star = bodyDesc(muStar, 7.0e8, {}, {});
        BodyDesc planet = bodyDesc(4.0e14, 6.4e6, {rOrbit, 0.0, 0.0}, {0.0, vOrbit, 0.0});
        ResolvedScene res = sceneOf({star, planet});
        const BodyId planetId = res.bodyIds[1];
        const double vRel = 1.0e5, d0 = 1.0e9;
        res.shuttles.push_back(resolvedShuttle(7, {rOrbit - d0, 0.0, 0.0}, {vRel, vOrbit, 0.0}));
        res.shuttles.back().desc.radius = 2.0;

        SimulationOptions opt;
        opt.recordShuttlePaths = true;
        Simulation sim(res, SceneSettings{}, opt);
        REQUIRE(sim.advanceTo(2.0e4));   // the flight takes about 1e4 s
        REQUIRE(sim.hasBodyStep());
        const double bodyStep = sim.bodyStepEnd() - sim.bodyStepStart();
        std::printf("SWEPT body step %.3e s, shuttle steps %ld\n", bodyStep, sim.stats().shuttleSteps);
        CHECK(bodyStep > 1.0e5);   // one body step is many times longer than the whole flight

        REQUIRE(sim.shuttles().alive[0] == 0);
        const std::vector<Event> ev = sim.drainEvents();
        REQUIRE(countOf(ev, EventType::ShuttleDestroyed) == 1);
        const Event* dead = firstOf(ev, EventType::ShuttleDestroyed);
        CHECK(dead->id == 7);
        CHECK(dead->other == planetId);

        // Time of the hit: radial infall from d0 at vRel under the planet's pull (energy: v^2 = vRel^2 + 2 mu (1/r - 1/d0),
        // integrated here with small steps); the star's tidal field over 1e4 s moves the shuttle by ~10 m, nothing.
        const double mu = 4.0e14, contact = 6.4e6 + 2.0;
        double r = d0, v = vRel, t = 0.0;
        while (r > contact) {   // symplectic Euler is plenty: dt = 0.01 s over a 1e4 s flight is 1e6 steps -> use 0.1 s
            const double dt = 0.1;
            v += mu / (r * r) * dt;
            r -= v * dt;
            t += dt;
        }
        std::printf("SWEPT event t = %.3f s, reference %.3f s\n", dead->t, t);
        CHECK(std::fabs(dead->t - t) < 0.1);   // measured 0.02 s: first-order reference with a 0.1 s step, chord of the last shuttle step

        // Not a lucky sample: the shuttle's accepted steps jump over the planet (a step longer than its diameter) and
        // no accepted step ends inside it, so a test on the end points alone would have missed the hit
        const auto& path = sim.shuttlePath(0);
        REQUIRE(path.size() >= 3);
        double longest = 0.0, innermost = 1e30;
        for (std::size_t k = 0; k + 1 < path.size(); ++k) {   // the last sample is the contact point itself
            Vec3 pc;
            REQUIRE(sim.bodyPositionAt(planetId, path[k].t, pc));
            innermost = std::min(innermost, norm(path[k].r - pc));
            if (k + 2 < path.size()) longest = std::max(longest, norm(path[k + 1].r - path[k].r));
        }
        std::printf("SWEPT longest accepted step %.3e m (planet diameter 1.28e7), innermost sample %.3e m\n", longest, innermost);
        CHECK(innermost > contact);
        // The hit is on the surface of the planet at the event time
        Vec3 pe;
        REQUIRE(sim.bodyPositionAt(planetId, dead->t, pe));
        CHECK(std::fabs(norm(shuttlePos(sim.shuttles(), 0) - pe) - contact) < 0.01 * contact);
    }

    TEST_CASE("Lucerna without collisions: energy, momentum and angular momentum stay at IAS15 precision")
    {
        scene_test::Lucerna L;
        SceneSettings settings;
        Simulation sim(L.res, settings);
        const Conserved c0 = sim.conservedWithMerges();
        const double domusYear = 2.0 * kPi * std::sqrt(std::pow(L.body("Domus").orbit->a, 3) / L.body("Lucerna").mu);
        REQUIRE(sim.advanceTo(0.5 * domusYear));
        CHECK(sim.stats().merges == 0);
        const Conserved c1 = sim.conservedWithMerges();
        const double dE = std::fabs(c1.energyG - c0.energyG) / std::fabs(c0.energyG);
        // Scales for P and L: the sum of |mu v| and |mu r x v| over the bodies
        double pScale = 0.0, lScale = 0.0;
        for (std::size_t i = 0; i < sim.bodies().size(); ++i) {
            pScale += sim.bodies().mu[i] * norm(bodyVel(sim.bodies(), i));
            lScale += sim.bodies().mu[i] * norm(cross(bodyPos(sim.bodies(), i), bodyVel(sim.bodies(), i)));
        }
        const double dP = norm(c1.momentumG - c0.momentumG) / pScale;
        const double dL = norm(c1.angularMomentumG - c0.angularMomentumG) / lScale;
        std::printf("LUCERNA half year: steps %ld, dE/E %.3e, dP/P %.3e, dL/L %.3e\n", sim.stats().bodySteps, dE, dP, dL);
        CHECK(sim.stats().bodySteps > 1000);
        CHECK(dE < 1e-13);   // measured 1.4e-15
        CHECK(dP < 1e-14);   // measured 8e-17
        CHECK(dL < 1e-14);
    }
}

TEST_SUITE("simulation edge cases")
{
    TEST_CASE("a lone body (no forces) and an empty scene do not hang")
    {
        ResolvedScene one = sceneOf({bodyDesc(1.0e14, 1.0e6, {1.0e9, 0.0, 0.0}, {10.0, 0.0, 0.0})});
        Simulation sim(one, SceneSettings{});
        const bool ok = sim.advanceTo(1.0e4);
        std::printf("LONE body ok=%d failed=%d t=%.6e steps=%ld\n", int(ok), int(sim.failed()), sim.time(), sim.stats().bodySteps);
        if (ok) CHECK(sim.time() >= 1.0e4);
        else CHECK(sim.failed());

        ResolvedScene none;
        Simulation empty(none, SceneSettings{});
        CHECK(empty.advanceTo(5.0));
        CHECK(empty.time() == 5.0);
    }
}
