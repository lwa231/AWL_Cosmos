#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "core/diagnostics.h"
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// The headless simulation runner (plan 4.4-4.7, 5.4, 6.3, 6.4, 8 items 1-3): construction from a scene, merges, shuttle
// destruction and atmosphere entry, conservation and the swept test at very large body steps.

TEST_SUITE("simulation")
{
    TEST_CASE("construction: bodies, shuttles, controllers, IDs")
    {
        scene_test::Lucerna L;
        ResolvedScene res = std::move(L.res);
        const BodyId domusId = res.bodyIds[L.sceneIndex("Domus")];
        res.shuttles.push_back(resolvedShuttle(101, {1.0e11, 0.0, 0.0}, {0.0, 1.0e3, 0.0}, "Prograde"));
        res.shuttles.push_back(resolvedShuttle(102, {1.1e11, 0.0, 0.0}, {0.0, 1.0e3, 0.0}, "wander"));   // case-insensitive
        res.shuttles.push_back(resolvedShuttle(103, {1.2e11, 0.0, 0.0}, {0.0, 1.0e3, 0.0}, "Teleport")); // unknown: skipped
        res.shuttles.push_back(resolvedShuttle(domusId, {1.3e11, 0.0, 0.0}, {0.0, 1.0e3, 0.0}));        // collides with a body ID
        res.shuttles.push_back(resolvedShuttle(101, {1.4e11, 0.0, 0.0}, {0.0, 1.0e3, 0.0}));            // duplicate shuttle ID

        SceneSettings settings;
        settings.infiniteFuel = true;
        settings.controllerTick_s = 0.25;
        Simulation sim(res, settings);

        CHECK(sim.bodies().size() == 14);
        REQUIRE(sim.shuttles().size() == 4);
        CHECK(sim.warnings().size() >= 3);
        CHECK(sim.infiniteFuel());
        CHECK(sim.controllerTick() == 0.25);
        CHECK(sim.time() == 0.0);
        CHECK(!sim.hasBodyStep());

        const Shuttles& sh = sim.shuttles();
        CHECK(sh.controller[0] == ControllerKind::Prograde);
        CHECK(sh.controller[1] == ControllerKind::Wander);
        CHECK(sh.consistent());
        // IDs: the ones given when free, fresh ones above every ID of the scene otherwise, never a body ID, all distinct
        CHECK(sh.ids[0] == 101);
        CHECK(sh.ids[1] == 102);
        for (std::size_t i = 0; i < sh.size(); ++i) {
            CHECK(!sim.bodies().find(sh.ids[i]).has_value());
            for (std::size_t j = i + 1; j < sh.size(); ++j) CHECK(sh.ids[i] != sh.ids[j]);
        }
        const std::uint64_t a = sim.allocateId(), b = sim.allocateId();
        CHECK(a != b);
        for (std::size_t i = 0; i < sh.size(); ++i) {
            CHECK(a != sh.ids[i]);
            CHECK(b != sh.ids[i]);
        }
        CHECK(!sim.bodies().find(a).has_value());
        // addShuttle takes a fresh ID, rejects a used one
        ShuttleDesc d;
        d.position = {1.5e11, 0.0, 0.0};
        d.velocity = {0.0, 1.0e3, 0.0};
        const ShuttleId n1 = sim.addShuttle(d);
        CHECK(n1 != 0);
        CHECK(n1 > b);
        CHECK(sim.shuttles().size() == 5);
        d.id = n1;
        CHECK(sim.addShuttle(d) == 0);
        d.id = 1; // a body ID
        CHECK(sim.addShuttle(d) == 0);

        // Controller choice: takes a live shuttle, rejects an unknown ID
        CHECK(sim.setController(101, ControllerKind::Retrograde));
        CHECK(sim.shuttles().controller[0] == ControllerKind::Retrograde);
        CHECK(!sim.setController(999999, ControllerKind::Coast));
    }

    TEST_CASE("advanceTo: runs ahead of the target on its own step grid and never clips a step to it")
    {
        // Two runs with different target sequences take the same body steps (plan 4.4)
        scene_test::Lucerna L;
        SceneSettings settings;
        SimulationOptions opt;
        Simulation a(L.res, settings, opt), b(L.res, settings, opt);
        const double T = 3.0e4;
        CHECK(a.advanceTo(T));
        CHECK(a.time() >= T);
        CHECK(a.targetTime() == T);
        CHECK(a.hasBodyStep());
        CHECK(a.bodyStepEnd() == a.time());
        CHECK(a.bodyStepStart() < T);
        for (double t = 1000.0; t < T + 1.0; t += 1000.0) CHECK(b.advanceTo(t));
        CHECK(b.advanceTo(a.time()));
        CHECK(a.time() == b.time());
        CHECK(a.stats().bodySteps == b.stats().bodySteps);
        CHECK(sameBits(a.bodies().state, b.bodies().state));
        // Already there: nothing happens
        const long steps = a.stats().bodySteps;
        CHECK(a.advanceTo(T / 2));
        CHECK(a.stats().bodySteps == steps);
        CHECK(a.targetTime() == T / 2);
    }

    TEST_CASE("dense output of the latest step and the shuttle's recorded path")
    {
        scene_test::Lucerna L;
        const std::size_t domus = L.storeIndex("Domus");
        ResolvedScene res = std::move(L.res);
        Vec3 pos, vel;
        circularOrbit(res.bodies.mu[domus], 3.0e6, 0.3, bodyPos(res.bodies, domus), bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(resolvedShuttle(501, pos, vel));
        SimulationOptions opt;
        opt.ias15.dtMax = 1500.0;
        opt.recordShuttlePaths = true;
        Simulation sim(res, SceneSettings{}, opt);
        REQUIRE(sim.advanceTo(4000.0));

        const BodyId id = sim.bodies().ids[domus];
        Vec3 p, pStart;
        REQUIRE(sim.bodyPositionAt(id, sim.bodyStepEnd(), p));
        CHECK(norm(p - bodyPos(sim.bodies(), domus)) <= 1e-3); // the end of the dense output is the committed state
        REQUIRE(sim.bodyPositionAt(id, sim.bodyStepStart(), pStart));
        CHECK(norm(pStart - p) > 0.0);
        Vec3 v;
        REQUIRE(sim.bodyVelocityAt(id, sim.bodyStepEnd(), v));
        CHECK(norm(v - bodyVel(sim.bodies(), domus)) <= 1e-9);
        CHECK(!sim.bodyPositionAt(123456789, sim.bodyStepEnd(), p));
        CHECK(sim.bodyDenseIds().size() == sim.bodies().size());

        // The shuttle: the recorded path covers the latest body step; the interpolation passes through its samples
        const ShuttleLastStep& ls = sim.shuttleStep(0);
        CHECK(ls.to.t == sim.time());
        CHECK(ls.from.t < ls.to.t);
        CHECK(norm(ls.to.r - shuttlePos(sim.shuttles(), 0)) == 0.0);
        const auto& path = sim.shuttlePath(0);
        REQUIRE(path.size() >= 2);
        CHECK(path.front().t == sim.bodyStepStart());
        CHECK(path.back().t == sim.time());
        for (const ShuttleSample& s : path) CHECK(norm(sim.shuttlePositionAt(0, s.t) - s.r) <= 1e-9);
        // Between samples the cubic Hermite follows the orbit: the interpolant through every other sample reproduces the
        // sample in between (the sample is not an input of it). Hermite error r theta^4 / 384 for the 2-step arc.
        REQUIRE(path.size() >= 4);
        double worst = 0.0;
        for (std::size_t k = 0; k + 2 < path.size(); ++k) {
            const Vec3 h = hermitePosition(path[k], path[k + 2], path[k + 1].t);
            worst = std::max(worst, norm(h - path[k + 1].r));
        }
        std::printf("PATH hermite worst midpoint miss %.3e m, %zu samples\n", worst, path.size());
        CHECK(worst < 1.0);
    }

    TEST_CASE("Hermite interpolation: exact for a cubic, clamped, derivative consistent")
    {
        // x(t) = 2 + 3t - t^2 + 0.5 t^3 on [1, 4]
        auto x = [](double t) { return 2.0 + 3.0 * t - t * t + 0.5 * t * t * t; };
        auto v = [](double t) { return 3.0 - 2.0 * t + 1.5 * t * t; };
        const ShuttleSample a{1.0, {x(1.0), 0.0, 0.0}, {v(1.0), 0.0, 0.0}};
        const ShuttleSample b{4.0, {x(4.0), 0.0, 0.0}, {v(4.0), 0.0, 0.0}};
        for (double t : {1.0, 1.7, 2.5, 3.9, 4.0}) {
            CHECK(hermitePosition(a, b, t).x == doctest::Approx(x(t)).epsilon(1e-13));
            CHECK(hermiteVelocity(a, b, t).x == doctest::Approx(v(t)).epsilon(1e-13));
        }
        CHECK(hermitePosition(a, b, 0.0).x == a.r.x);   // clamped
        CHECK(hermitePosition(a, b, 9.0).x == b.r.x);
        CHECK(hermitePosition(a, a, 1.0).x == a.r.x);   // zero-length interval
    }

    TEST_CASE("two bodies colliding in the runner merge: momentum conserved, the right events, energy with the merge added back")
    {
        BodyDesc A = bodyDesc(4.0e14, 6.4e6, {-5.0e8, 0.0, 0.0}, {5.0e3, 0.0, 0.0});
        BodyDesc B = bodyDesc(4.0e13, 3.0e6, {5.0e8, 0.0, 0.0}, {-5.0e3, 0.0, 0.0});
        BodyDesc C = bodyDesc(1.0e12, 1.0e5, {0.0, 3.0e9, 0.0}, {-2.0e2, 0.0, 0.0}); // a bystander
        ResolvedScene res = sceneOf({A, B, C});
        const BodyId idA = res.bodyIds[0], idB = res.bodyIds[1], idC = res.bodyIds[2];

        SimulationOptions opt;
        opt.ias15.dtMax = 100.0;
        Simulation sim(res, SceneSettings{}, opt);
        const Conserved c0 = sim.conserved();
        const double mu0 = sim.bodies().mu[0] + sim.bodies().mu[1];

        // Fly until the merge happens
        while (sim.stats().merges == 0 && sim.time() < 4.0e5) REQUIRE(sim.advanceTo(sim.time() + 1.0));
        REQUIRE(sim.stats().merges == 1);
        CHECK(sim.stats().bodyStepRedos >= 1);
        REQUIRE(sim.mergeLog().size() == 1);
        const MergeResult m = sim.mergeLog()[0];
        CHECK(m.survivor == idA);   // the heavier one
        CHECK(m.absorbed == idB);
        CHECK(sim.time() == m.t);
        CHECK(m.t > 5.0e4);
        CHECK(m.t < 1.5e5);
        CHECK(m.relSpeed > 1.0e4);

        // The store: the survivor with the combined mu, the absorbed one gone for good
        CHECK(sim.bodies().size() == 2);
        CHECK(sim.bodies().find(idA).has_value());
        CHECK(!sim.bodies().find(idB).has_value());
        CHECK(sim.bodies().find(idC).has_value());
        const std::size_t iA = *sim.bodies().find(idA);
        CHECK(sim.bodies().mu[iA] == mu0);
        CHECK(sim.bodies().radius[iA] == doctest::Approx(std::cbrt(6.4e6 * 6.4e6 * 6.4e6 + 3.0e6 * 3.0e6 * 3.0e6)));

        // Events: Collision (lower ID first), then Merge, at the merge time; no reclassification (planet + planet)
        const std::vector<Event> ev = sim.drainEvents();
        REQUIRE(ev.size() == 2);
        CHECK(ev[0].type == EventType::Collision);
        CHECK(ev[0].id == std::min(idA, idB));
        CHECK(ev[0].other == std::max(idA, idB));
        CHECK(ev[0].t == m.t);
        CHECK(ev[0].relSpeed == m.relSpeed);
        CHECK(ev[1].type == EventType::Merge);
        CHECK(ev[1].id == idA);
        CHECK(ev[1].other == idB);
        CHECK(ev[1].t == m.t);
        CHECK(sim.drainEvents().empty());

        // Momentum: the merge conserves it (to rounding), and so does the integration before it
        const Conserved c1 = sim.conserved();
        const double pScale = 4.4e14 * 5.0e3;
        std::printf("MERGE dP/pScale = %.3e, t = %.6e, redos %ld\n", norm(c1.momentumG - c0.momentumG) / pScale, m.t, sim.stats().bodyStepRedos);
        CHECK(norm(c1.momentumG - c0.momentumG) <= 1e-12 * pScale);

        // The collision point lies on the line of centres between the two surfaces: within the contact distance of the new centre
        const double contact = 6.4e6 + 3.0e6;
        CHECK(norm(m.point - bodyPos(sim.bodies(), iA)) < contact);

        // Energy with the merge loss added back is the integrator's error only, and the ledger holds the loss
        CHECK(sim.ledger().energyG < 0.0); // inelastic: kinetic energy lost
        CHECK(sim.ledger().energyG == m.dEnergyG);
        const Conserved e1 = sim.conservedWithMerges();
        const double scale = std::fabs(c0.energyG);
        std::printf("MERGE dE/E = %.3e\n", (e1.energyG - c0.energyG) / scale);
        CHECK(std::fabs(e1.energyG - c0.energyG) <= 1e-9 * scale);

        // Carry on: the integrator restarts and the two remaining bodies move on in peace
        REQUIRE(sim.advanceTo(m.t + 2.0e4));
        CHECK(sim.stats().merges == 1);
        CHECK(std::fabs(sim.conservedWithMerges().energyG - c0.energyG) <= 1e-9 * scale);
        CHECK(norm(sim.conserved().momentumG - c0.momentumG) <= 1e-12 * pScale);
    }
}
