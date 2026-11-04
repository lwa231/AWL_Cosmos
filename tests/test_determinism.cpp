#include <doctest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
#include "core/atmosphere.h"
#include "core/rng.h"
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// Determinism (plan 4.6, 14): the result of a run is bit-identical for any thread count and any sequence of advanceTo
// targets. Lucerna with 200 shuttles under the four autopilots, some in the upper air of Domus so that drag, atmosphere
// entry and destruction all happen inside the compared span.

namespace {

struct Snapshot {
    BodyState bodies;
    std::vector<double> mu, radius, spin;
    std::vector<BodyId> ids;
    BodyState shuttles;
    std::vector<double> heading, spin_s, fuel, throttle, turn, nextStep;
    std::vector<std::uint8_t> alive, fuelEmptyReported;
    std::vector<std::int64_t> nextTick;
    std::vector<double> lastStepBits;   // from/to of every shuttle's last step
    std::vector<Event> events;          // everything drained, in delivery order
    long bodySteps = 0, shuttleSteps = 0, rejects = 0, ticks = 0, forceCalls = 0, destroyed = 0, merges = 0;
    double time = 0.0;
};

Snapshot snapshot(const Simulation& sim, std::vector<Event> events)
{
    Snapshot s;
    const Bodies& b = sim.bodies();
    s.bodies = b.state;
    s.mu = b.mu;
    s.radius = b.radius;
    s.spin = b.spinRate;
    s.ids = b.ids;
    const Shuttles& sh = sim.shuttles();
    s.shuttles = sh.state;
    s.heading = sh.heading;
    s.spin_s = sh.spin;
    s.fuel = sh.fuelMass;
    s.throttle = sh.throttle;
    s.turn = sh.turn;
    s.nextStep = sh.nextStep;
    s.alive = sh.alive;
    s.fuelEmptyReported = sh.fuelEmptyReported;
    s.nextTick = sh.nextTick;
    for (std::size_t i = 0; i < sh.size(); ++i) {
        const ShuttleLastStep& ls = sim.shuttleStep(i);
        for (const ShuttleSample* p : {&ls.from, &ls.to})
            for (double d : {p->t, p->r.x, p->r.y, p->r.z, p->v.x, p->v.y, p->v.z}) s.lastStepBits.push_back(d);
    }
    s.events = std::move(events);
    s.bodySteps = sim.stats().bodySteps;
    s.shuttleSteps = sim.stats().shuttleSteps;
    s.rejects = sim.stats().shuttleRejects;
    s.ticks = sim.stats().controllerTicks;
    s.forceCalls = sim.stats().shuttleForceCalls;
    s.destroyed = sim.stats().shuttlesDestroyed;
    s.merges = sim.stats().merges;
    s.time = sim.time();
    return s;
}

// Names the first thing that differs ("" when identical).
std::string firstDifference(const Snapshot& a, const Snapshot& b)
{
    if (!sameBits(a.time, b.time)) return "time";
    if (a.ids != b.ids) return "body ids";
    if (!sameBits(a.bodies, b.bodies)) return "body state";
    if (!sameBits(a.mu, b.mu) || !sameBits(a.radius, b.radius) || !sameBits(a.spin, b.spin)) return "body properties";
    if (!sameBits(a.shuttles, b.shuttles)) return "shuttle state";
    if (!sameBits(a.heading, b.heading) || !sameBits(a.spin_s, b.spin_s)) return "shuttle heading/spin";
    if (!sameBits(a.fuel, b.fuel)) return "shuttle fuel";
    if (!sameBits(a.throttle, b.throttle) || !sameBits(a.turn, b.turn)) return "shuttle command";
    if (!sameBits(a.nextStep, b.nextStep)) return "shuttle step proposal";
    if (a.alive != b.alive || a.fuelEmptyReported != b.fuelEmptyReported || a.nextTick != b.nextTick) return "shuttle flags";
    if (!sameBits(a.lastStepBits, b.lastStepBits)) return "shuttle last step";
    if (a.events.size() != b.events.size()) return "event count";
    for (std::size_t i = 0; i < a.events.size(); ++i)
        if (!sameBits(a.events[i], b.events[i])) return "event " + std::to_string(i);
    if (a.bodySteps != b.bodySteps) return "body steps";
    if (a.shuttleSteps != b.shuttleSteps || a.rejects != b.rejects || a.ticks != b.ticks || a.forceCalls != b.forceCalls)
        return "shuttle statistics";
    if (a.destroyed != b.destroyed || a.merges != b.merges) return "destroyed/merges";
    return "";
}

// Lucerna plus 200 shuttles. Shuttle j: host body, orbit radius, direction, phase and controller come from the counter
// RNG of (j), so the set is the same everywhere.
ResolvedScene lucernaWithShuttles(scene_test::Lucerna& L, int count)
{
    ResolvedScene res = L.res;
    const char* names[] = {"Domus", "Maxima", "Rubor", "Velum", "Lucerna"};
    const char* controllers[] = {"Coast", "Prograde", "Retrograde", "Wander"};
    for (int j = 0; j < count; ++j) {
        Rng rng = Rng::forTick(static_cast<std::uint64_t>(j) + 1, 99);
        const bool lowDomus = j < count / 5;   // 40 shuttles in the top of Domus's atmosphere (and just above it)
        const std::size_t host = L.storeIndex(lowDomus ? "Domus" : names[1 + rng.below(4)]);
        const double R = res.bodies.radius[host];
        double r;
        const char* controller;
        if (lowDomus) {
            r = R + rng.uniform(5.0e4, 1.2e5);   // atmosphere top is at 98 km: part start inside, part enter later
            controller = (j % 2 == 0) ? "Retrograde" : "Coast";   // the Coast ones fall (and burn up) within ~150 s
        } else {
            r = R * rng.uniform(1.5, 6.0);
            controller = controllers[rng.below(4)];
        }
        const double phi = rng.uniform(0.0, 2.0 * kPi);
        const double dir = rng.uniform() < 0.5 ? 1.0 : -1.0;
        const double speed = std::sqrt(res.bodies.mu[host] / r) * (lowDomus ? rng.uniform(0.3, 0.9) : rng.uniform(0.75, 1.0));
        const Vec3 pb = bodyPos(res.bodies, host), vb = bodyVel(res.bodies, host);
        const Vec3 pos = pb + Vec3{r * std::cos(phi), r * std::sin(phi), 0.0};
        const Vec3 vel = vb + Vec3{-dir * speed * std::sin(phi), dir * speed * std::cos(phi), 0.0};
        res.shuttles.push_back(resolvedShuttle(1000 + static_cast<std::uint64_t>(j), pos, vel, controller));
    }
    return res;
}

struct RunSpec {
    int threads = 1;
    std::vector<double> targets;     // advanceTo targets in order
    int switchThreadsTo = 0;         // if > 0: change the thread count after half of the targets
};

Snapshot run(const ResolvedScene& res, const RunSpec& spec, const SimulationOptions& base)
{
    SimulationOptions opt = base;
    opt.threads = spec.threads;
    Simulation sim(res, SceneSettings{}, opt);
    std::vector<Event> events;
    for (std::size_t k = 0; k < spec.targets.size(); ++k) {
        if (spec.switchThreadsTo > 0 && k == spec.targets.size() / 2) sim.setThreadCount(spec.switchThreadsTo);
        REQUIRE(sim.advanceTo(spec.targets[k]));
        const std::vector<Event> ev = sim.drainEvents();
        events.insert(events.end(), ev.begin(), ev.end());
    }
    return snapshot(sim, std::move(events));
}

std::vector<double> evenTargets(double span, int n)
{
    std::vector<double> t;
    for (int k = 1; k <= n; ++k) t.push_back(span * k / n);
    return t;
}

} // namespace

TEST_SUITE("determinism")
{
    TEST_CASE("Lucerna + 200 shuttles: bit-identical for 1, 4 and 16 threads and for different advanceTo schedules")
    {
        scene_test::Lucerna L;
        const int count = 200;
        const ResolvedScene res = lucernaWithShuttles(L, count);

        // Short body steps (dtMax) so that the span holds many of them; the shuttles run at the controller tick (0.1 s)
        SimulationOptions base;
        base.ias15.dtMax = 10.0;
        const double span = 300.0;   // the sub-orbital Retrograde shuttles over Domus reach the air and the ground within it

        const auto t0 = std::chrono::steady_clock::now();
        // Reference: one thread, 100 small targets (each takes one body step or none, the rest is the step grid running ahead)
        RunSpec ref;
        ref.threads = 1;
        ref.targets = evenTargets(span, 100);
        const Snapshot a = run(res, ref, base);
        const double refSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        // The reference is a real scenario: shuttles fly, burn, enter the air, die
        const long total = static_cast<long>(a.alive.size());
        REQUIRE(total == count);
        long alive = 0;
        for (std::uint8_t v : a.alive) alive += v;
        std::printf("DETERMINISM reference: %.1f s wall, %ld body steps, %ld shuttle steps (%ld rejected), %ld ticks, %ld destroyed, %ld alive of %ld\n",
                    refSeconds, a.bodySteps, a.shuttleSteps, a.rejects, a.ticks, a.destroyed, alive, total);
        CHECK(a.time >= span);
        CHECK(a.bodySteps >= 25);
        CHECK(a.shuttleSteps > 50000);
        CHECK(a.ticks > 10000);
        CHECK(alive > 100);
        CHECK(alive < total);
        CHECK(countOf(a.events, EventType::AtmosphereEntry) >= 5);
        CHECK(countOf(a.events, EventType::ShuttleDestroyed) >= 5);
        CHECK(static_cast<long>(countOf(a.events, EventType::ShuttleDestroyed)) == a.destroyed);
        CHECK(std::is_sorted(a.events.begin(), a.events.end(), eventBefore));
        double burned = 0.0;
        for (std::size_t i = 0; i < a.fuel.size(); ++i) burned += 3000.0 - a.fuel[i];
        CHECK(burned > 1000.0);   // the burners really burned

        struct Variant {
            const char* name;
            RunSpec spec;
        };
        std::vector<Variant> variants;
        variants.push_back({"4 threads, 100 small targets", {4, evenTargets(span, 100), 0}});
        variants.push_back({"16 threads, 100 small targets", {16, evenTargets(span, 100), 0}});
        variants.push_back({"3 threads, 3 large targets", {3, {0.37 * span, 0.66 * span, span}, 0}});
        variants.push_back({"16 threads, one target", {16, {span}, 0}});
        variants.push_back({"2 threads then 8, 7 uneven targets", {2, {13.0, 77.0, 78.0, 190.0, 191.5, 250.0, span}, 8}});
        for (const Variant& v : variants) {
            const Snapshot b = run(res, v.spec, base);
            const std::string diff = firstDifference(a, b);
            INFO("variant: " << v.name << ", first difference: " << diff);
            CHECK(diff.empty());
        }
    }

    TEST_CASE("targets that do not move the clock (earlier than the current time) change nothing")
    {
        scene_test::Lucerna L;
        const ResolvedScene res = lucernaWithShuttles(L, 20);
        SimulationOptions base;
        base.ias15.dtMax = 20.0;
        RunSpec plain, jittery;
        plain.targets = {100.0, 200.0};
        jittery.targets = {100.0, 40.0, 100.0, 0.0, 150.0, 99.0, 200.0, 200.0};
        const Snapshot a = run(res, plain, base);
        const Snapshot b = run(res, jittery, base);
        CHECK(firstDifference(a, b).empty());
    }

    TEST_CASE("with a merge: clipped step, IAS15 reset and the shuttle pass are thread- and schedule-independent")
    {
        // Two bodies on a collision course, a bystander, and shuttles around the bodies (some are swallowed)
        BodyDesc A = bodyDesc(4.0e14, 6.4e6, {-6.0e6, 0.0, 0.0}, {5.0e3, 0.0, 0.0});
        BodyDesc B = bodyDesc(4.0e13, 3.0e6, {6.0e6, 0.0, 0.0}, {-5.0e3, 0.0, 0.0});
        BodyDesc C = bodyDesc(1.0e12, 1.0e5, {0.0, 3.0e7, 0.0}, {-2.0e2, 0.0, 0.0});
        ResolvedScene res = sceneOf({A, B, C});
        const char* controllers[] = {"Coast", "Prograde", "Retrograde", "Wander"};
        for (int j = 0; j < 40; ++j) {
            Rng rng = Rng::forTick(static_cast<std::uint64_t>(j) + 1, 5);
            const std::size_t host = rng.below(2);
            const double r = res.bodies.radius[host] * rng.uniform(1.3, 4.0);
            const double phi = rng.uniform(0.0, 2.0 * kPi);
            const double dir = rng.uniform() < 0.5 ? 1.0 : -1.0;
            const double speed = std::sqrt(res.bodies.mu[host] / r) * rng.uniform(0.6, 1.0);
            const Vec3 pos = bodyPos(res.bodies, host) + Vec3{r * std::cos(phi), r * std::sin(phi), 0.0};
            const Vec3 vel = bodyVel(res.bodies, host) + Vec3{-dir * speed * std::sin(phi), dir * speed * std::cos(phi), 0.0};
            res.shuttles.push_back(resolvedShuttle(100 + static_cast<std::uint64_t>(j), pos, vel, controllers[rng.below(4)]));
        }
        SimulationOptions base;
        base.ias15.dtMax = 10.0;
        const double span = 400.0;   // the bodies touch after about 200 s

        RunSpec ref;
        ref.threads = 1;
        ref.targets = evenTargets(span, 40);
        const Snapshot a = run(res, ref, base);
        CHECK(a.merges == 1);
        CHECK(countOf(a.events, EventType::Merge) == 1);
        CHECK(countOf(a.events, EventType::Collision) == 1);
        CHECK(a.bodies.x.size() == 2);

        for (const RunSpec& spec : {RunSpec{8, evenTargets(span, 40), 0}, RunSpec{5, {120.0, 190.0, 215.0, span}, 0},
                                    RunSpec{16, {span}, 0}}) {
            const Snapshot b = run(res, spec, base);
            const std::string diff = firstDifference(a, b);
            INFO("threads " << spec.threads << ", first difference: " << diff);
            CHECK(diff.empty());
        }

        // Negative control: the comparison can fail. A different body-step limit is a different (equally valid) run.
        SimulationOptions other = base;
        other.ias15.dtMax = 10.5;
        const Snapshot c = run(res, ref, other);
        CHECK(!firstDifference(a, c).empty());
    }
}
