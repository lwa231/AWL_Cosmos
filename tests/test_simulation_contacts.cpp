#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// Body-body contacts at the edges of the step logic (plan 5.11, 5.4): a contact so close to the start of a step that the
// clipped step would not move the clock, and a chord that fires on sag alone. And the controller tick period changed
// between runs (plan 4.6).

namespace {

// Two 1 km bodies on a head-on course at 1 km/s with the given surface gap. mu is negligible.
ResolvedScene headOn(double gap)
{
    BodyDesc a = bodyDesc(1.0e3, 1000.0, {0.0, 0.0, 0.0}, {500.0, 0.0, 0.0});
    BodyDesc b = bodyDesc(1.0e3, 1000.0, {2000.0 + gap, 0.0, 0.0}, {-500.0, 0.0, 0.0});
    return sceneOf({a, b});
}

// A tiny moon (mu 1e5) on an e = 0.5 orbit about a planet (R 6.4e6) starting at apoapsis, periapsis `clearance` m above the
// contact distance R_a + R_b = 6.5e6 m. Returns the scene and the period.
ResolvedScene grazing(double clearance, double& period)
{
    const double muP = 4.0e14, Rp = 6.4e6, Rm = 1.0e5;
    const double rp = Rp + Rm + clearance, e = 0.5;
    const double a = rp / (1.0 - e), ra = a * (1.0 + e);
    const double vApo = std::sqrt(muP * (2.0 / ra - 1.0 / a));
    period = 2.0 * kPi * std::sqrt(a * a * a / muP);
    return sceneOf({bodyDesc(muP, Rp, {}, {}), bodyDesc(1.0e5, Rm, {ra, 0.0, 0.0}, {0.0, vApo, 0.0})});
}

// Records the tick of every call; asks for every tick.
struct TickRecorder final : IController {
    ShuttleCommand update(const ShuttleObservation& o, Rng&) override
    {
        ticks.push_back(o.tick);
        times.push_back(o.t_sim);
        return {};
    }
    std::vector<std::int64_t> ticks;
    std::vector<double> times;
};

} // namespace

TEST_SUITE("simulation contacts")
{
    TEST_CASE("a contact within 4 ulp of the clock after the step start merges instead of stalling (late sim time)")
    {
        // At t = 1e9 s, 4 ulp is 4.8e-7 s: a 0.2 mm gap closing at 1 km/s is touched after 2e-7 s. The clipped step would not
        // move the clock; the old test merged only contacts at exactly 0 and so repeated the same step forever.
        for (double gap : {5.0e-5, 1.0e-4, 2.0e-4, 1.0e-3, 1.0e-2}) {
            SimulationOptions opt;
            opt.startTime = 1.0e9;
            Simulation sim(headOn(gap), SceneSettings{}, opt);
            const bool ok = sim.advanceTo(opt.startTime + 10.0);
            INFO("gap " << gap);
            CHECK(ok);
            CHECK(!sim.failed());
            CHECK(sim.stats().merges == 1);
            CHECK(sim.bodies().size() == 1);
            CHECK(sim.time() >= opt.startTime + 10.0);
            REQUIRE(sim.mergeLog().size() == 1);
            // The merge is at the moment of touching, to the resolution of the clock (4 ulp = 4.8e-7 s) or the step grid
            CHECK(sim.mergeLog()[0].t - opt.startTime < 1.0e-3);
            CHECK(sim.mergeLog()[0].relSpeed == doctest::Approx(1000.0).epsilon(1e-9));
        }
    }

    TEST_CASE("a pair already overlapping at the start merges at once")
    {
        BodyDesc a = bodyDesc(1.0e3, 1000.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        BodyDesc b = bodyDesc(1.0e3, 1000.0, {1500.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        Simulation sim(sceneOf({a, b}), SceneSettings{});
        REQUIRE(sim.advanceTo(5.0));
        CHECK(sim.stats().merges == 1);
        REQUIRE(sim.mergeLog().size() == 1);
        CHECK(sim.mergeLog()[0].t == 0.0);
    }

    TEST_CASE("a near-grazing orbit that never touches is not merged because the chord of a long step does")
    {
        // Periapsis 300 m (5e-5 of the contact distance) above the surfaces. The chord between the ends of a step across
        // periapsis cuts inside the orbit by far more than that, so the swept test fires; the step is shortened, the pair is
        // tested again and found apart.
        double T = 0.0;
        const ResolvedScene res = grazing(300.0, T);
        SimulationOptions opt;
        opt.ias15.dtMax = 300.0;   // long steps across periapsis (the sag of a 300 s chord at 9.5 m/s2 is 100 km)
        Simulation sim(res, SceneSettings{}, opt);
        double minSep = std::numeric_limits<double>::infinity();
        while (sim.time() < 1.2 * T) {
            REQUIRE(sim.advanceTo(sim.time() + 1.0));
            if (sim.stats().merges != 0) break;
            minSep = std::min(minSep, norm(bodyPos(sim.bodies(), 0) - bodyPos(sim.bodies(), 1)));
        }
        std::printf("GRAZING clearance 300 m: merges %ld, redos %ld, steps %ld, min step-end separation %.1f m above contact\n",
                    sim.stats().merges, sim.stats().bodyStepRedos, sim.stats().bodySteps, minSep - 6.5e6);
        CHECK(sim.stats().bodyStepRedos >= 1);   // the chord did fire, so this test does test the retest
        CHECK(sim.stats().merges == 0);
        CHECK(minSep > 6.5e6);                   // and the bodies never reached each other at a step end
        CHECK(sim.bodies().size() == 2);
    }

    TEST_CASE("an orbit whose periapsis is inside the contact distance does merge, at periapsis")
    {
        double T = 0.0;
        const ResolvedScene res = grazing(-3000.0, T);
        SimulationOptions opt;
        opt.ias15.dtMax = 300.0;
        Simulation sim(res, SceneSettings{}, opt);
        REQUIRE(sim.advanceTo(0.8 * T));
        REQUIRE(sim.stats().merges == 1);
        REQUIRE(sim.mergeLog().size() == 1);
        const double tm = sim.mergeLog()[0].t;
        std::printf("GRAZING clearance -3 km: merge at %.6f T, redos %ld\n", tm / T, sim.stats().bodyStepRedos);
        CHECK(tm > 0.48 * T);   // periapsis is at T / 2
        CHECK(tm < 0.52 * T);
        CHECK(sim.bodies().size() == 1);
    }

    TEST_CASE("changing the controller tick period re-evaluates the controllers on the new grid at once")
    {
        // At t = 100 s the stored tick index of the old grid (0.1 s) is 1000, which on the new grid (1 s) is t = 1000 s: if
        // kept, no call would come for 900 s and the held command would go stale.
        TickRecorder rec;
        const double mu = 1.3e20;
        ResolvedScene res = sceneOf({bodyDesc(mu, 7.0e8, {}, {})});
        res.shuttles.push_back(resolvedShuttle(50, {1.0e10, 0.0, 0.0}, {0.0, std::sqrt(mu / 1.0e10), 0.0}, "Prograde"));
        SimulationOptions opt;
        opt.ias15.dtMax = 5.0;
        Simulation sim(res, SceneSettings{}, opt);
        sim.controllers().set(ControllerKind::Prograde, &rec);
        REQUIRE(sim.advanceTo(100.0));
        const std::size_t before = rec.ticks.size();
        CHECK(before > 900);   // every 0.1 s tick

        sim.setControllerTick(1.0);
        CHECK(sim.controllerTick() == 1.0);
        sim.setControllerTick(-3.0);   // not a period: ignored
        sim.setControllerTick(std::numeric_limits<double>::quiet_NaN());
        CHECK(sim.controllerTick() == 1.0);

        const double t1 = sim.time();
        REQUIRE(sim.advanceTo(t1 + 30.0));
        REQUIRE(rec.ticks.size() > before);
        // The first call after the change comes at the start of the next step, on the new grid
        CHECK(rec.times[before] < t1 + 6.0);   // within one body step (dtMax 5 s)
        CHECK(rec.ticks[before] == static_cast<std::int64_t>(std::floor(rec.times[before])));
        // and then once per second
        const std::size_t calls = rec.ticks.size() - before;
        CHECK(calls >= 28);
        CHECK(calls <= 32);
    }
}
