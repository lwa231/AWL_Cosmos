#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include "test_app_overlays_util.h"

using namespace ovl_test;

namespace {

PredictRequest requestFor(TargetKind kind, std::uint64_t id, double horizon, int samples)
{
    PredictRequest r;
    r.kind = kind;
    r.id = id;
    r.horizon_s = horizon;
    r.samples = samples;
    return r;
}

// Everything the live simulation holds that a stray worker could change, bit for bit.
bool sameSimulation(const Simulation& a, const Simulation& b)
{
    if (!sim_test::sameBits(a.time(), b.time()) || a.bodies().size() != b.bodies().size() ||
        a.shuttles().size() != b.shuttles().size() || a.stats().bodySteps != b.stats().bodySteps)
        return false;
    if (!sim_test::sameBits(a.bodies().state, b.bodies().state) || !sim_test::sameBits(a.shuttles().state, b.shuttles().state)) return false;
    if (!sim_test::sameBits(a.shuttles().heading, b.shuttles().heading) || !sim_test::sameBits(a.shuttles().fuelMass, b.shuttles().fuelMass)) return false;
    const std::vector<Event>& ea = a.pendingEvents().events();
    const std::vector<Event>& eb = b.pendingEvents().events();
    if (ea.size() != eb.size()) return false;
    for (std::size_t i = 0; i < ea.size(); ++i)
        if (!sim_test::sameBits(ea[i], eb[i])) return false;
    return true;
}

} // namespace

TEST_SUITE("app overlays predict")
{
    TEST_CASE("a predicted shuttle path equals a direct propagation, sample for sample")
    {
        ResolvedScene res = driver_test::lucernaWithShuttles(2);       // 900 coasts, 901 is a Prograde autopilot
        SimDriver live(res, SceneSettings{});
        live.advanceTo(120.0);                                          // not at the start: the copy starts from a running state

        const std::uint64_t id = 900;                                   // a coasting shuttle: nothing the live autopilot's tick phase can change
        const double horizon = 1800.0;
        const int samples = 60;
        const PredictedPath p = predictNow(live.sim(), requestFor(TargetKind::Shuttle, id, horizon, samples));
        REQUIRE(p.valid());
        REQUIRE(p.count == static_cast<std::size_t>(samples) + 1);
        CHECK_FALSE(p.truncated);
        CHECK_FALSE(p.targetEnded);
        CHECK(p.t0 == live.sim().time());

        // The reference: a second identical simulation, advanced directly to each sample time.
        SimDriver ref(res, SceneSettings{});
        ref.advanceTo(120.0);
        REQUIRE(sim_test::sameBits(ref.sim().time(), p.t0));
        const auto idx = ref.sim().shuttles().find(id);
        REQUIRE(idx.has_value());
        double worst = 0.0, bodyWorst = 0.0;
        for (std::size_t k = 0; k <= static_cast<std::size_t>(samples); ++k) {
            const double tk = p.timeOf(k);
            if (k > 0) ref.sim().advanceTo(tk);
            const Vec3 truth = k == 0 ? sim_test::shuttlePos(ref.sim().shuttles(), *idx) : ref.sim().shuttlePositionAt(*idx, tk);
            worst = std::max(worst, std::hypot(p.target[k].x - truth.x, p.target[k].y - truth.y));
            // Bodies too (the path can be re-expressed in any frame).
            for (std::size_t j = 0; j < p.bodyIds.size(); ++j) {
                Vec3 b;
                if (k == 0) {
                    const std::size_t bi = *ref.sim().bodies().find(p.bodyIds[j]);
                    b = sim_test::bodyPos(ref.sim().bodies(), bi);
                } else {
                    REQUIRE(ref.sim().bodyPositionAt(p.bodyIds[j], tk, b));
                }
                bodyWorst = std::max(bodyWorst, std::hypot(p.bodies[k * p.bodyIds.size() + j].x - b.x, p.bodies[k * p.bodyIds.size() + j].y - b.y));
            }
        }
        // The copy starts a fresh IAS15 at t0 (no step history), so it agrees with the continuing run to integrator accuracy,
        // not bit for bit: millimetres here.
        CHECK(worst < 0.05);
        CHECK(bodyWorst < 1.0e-3);
        // The shuttle really moved (a path of nothing would pass the test too).
        CHECK(std::hypot(p.target.back().x - p.target.front().x, p.target.back().y - p.target.front().y) > 1.0e5);
        // And the barycentre column is the mu-weighted mean of the bodies.
        double m = 0.0, bx = 0.0;
        for (std::size_t j = 0; j < p.bodyIds.size(); ++j) {
            const double mu = live.sim().bodies().mu[*live.sim().bodies().find(p.bodyIds[j])];
            m += mu;
            bx += mu * p.bodies[10 * p.bodyIds.size() + j].x;
        }
        CHECK(p.barycentre[10].x == doctest::Approx(bx / m).epsilon(1e-9));
    }

    TEST_CASE("a predicted body path equals a direct propagation")
    {
        SunPlanetMoon w;
        SimDriver live(w.res, SceneSettings{});
        live.advanceTo(5000.0);
        const PredictedPath p = predictNow(live.sim(), requestFor(TargetKind::Body, w.moon, w.moonPeriod, 100));
        REQUIRE(p.valid());
        SimDriver ref(w.res, SceneSettings{});
        ref.advanceTo(5000.0);
        for (std::size_t k = 1; k <= 100; ++k) {
            ref.sim().advanceTo(p.timeOf(k));
            Vec3 b;
            REQUIRE(ref.sim().bodyPositionAt(w.moon, p.timeOf(k), b));
            REQUIRE(std::hypot(p.target[k].x - b.x, p.target[k].y - b.y) < 0.01);   // fresh IAS15 vs the continuing run
        }
        // One period later the moon is back near where it started, relative to the planet (a closed relative orbit).
        const int cp = p.bodyColumn(w.planet);
        REQUIRE(cp >= 0);
        const Vec3 pl0 = p.bodies[0 * p.bodyIds.size() + static_cast<std::size_t>(cp)];
        const Vec3 plN = p.bodies[100 * p.bodyIds.size() + static_cast<std::size_t>(cp)];
        const double r0 = std::hypot(p.target[0].x - pl0.x, p.target[0].y - pl0.y);
        const double rN = std::hypot(p.target[100].x - plN.x, p.target[100].y - plN.y);
        CHECK(rN == doctest::Approx(r0).epsilon(0.05));
        CHECK(std::hypot((p.target[100].x - plN.x) - (p.target[0].x - pl0.x), (p.target[100].y - plN.y) - (p.target[0].y - pl0.y)) <
              0.3 * r0);
    }

    TEST_CASE("the worker returns the same path as the synchronous run and never touches the live simulation")
    {
        ResolvedScene res = driver_test::lucernaWithShuttles(2);
        SimDriver a(res, SceneSettings{}), b(res, SceneSettings{});     // a is asked for predictions, b never is
        a.advanceTo(60.0);
        b.advanceTo(60.0);
        REQUIRE(sameSimulation(a.sim(), b.sim()));

        PathPredictor pp;
        const PredictRequest req = requestFor(TargetKind::Shuttle, 901, 3600.0, 120);
        const std::uint64_t n = pp.request(a.sim(), req);
        CHECK(n == 1);
        // The frame goes on while the worker runs: the live simulation steps (a, with the worker running) and its twin (b).
        for (int i = 1; i <= 50; ++i) {
            a.advanceTo(60.0 + 40.0 * i);
            b.advanceTo(60.0 + 40.0 * i);
            pp.poll();                                                   // never blocks
        }
        REQUIRE(pp.waitIdle(60.0));
        if (!pp.result().valid()) REQUIRE(pp.poll());                    // (the loop above may already have taken it)
        CHECK_FALSE(pp.poll());                                          // handed over once
        const PredictedPath& got = pp.result();
        REQUIRE(got.valid());
        CHECK(got.requestId == n);

        // Bit-identical to the synchronous run from the same moment (a fresh twin at t = 60).
        SimDriver c(res, SceneSettings{});
        c.advanceTo(60.0);
        const PredictedPath sync = predictNow(c.sim(), req);
        CHECK(got.t0 == sync.t0);                                        // the snapshot was taken at request time
        REQUIRE(sync.count == got.count);
        for (std::size_t k = 0; k < got.count; ++k) {
            REQUIRE(sim_test::sameBits(got.target[k].x, sync.target[k].x));
            REQUIRE(sim_test::sameBits(got.target[k].y, sync.target[k].y));
        }
        // The live simulation is exactly where its never-asked twin is.
        CHECK(sameSimulation(a.sim(), b.sim()));
    }

    TEST_CASE("a new request supersedes the old one")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        d.advanceTo(100.0);
        PathPredictor pp;
        PredictRequest slow = requestFor(TargetKind::Body, w.moon, 50.0 * w.moonPeriod, 4000);
        slow.maxBodySteps = 100000000;                                  // would run for a long time
        const std::uint64_t first = pp.request(d.sim(), slow);
        const std::uint64_t second = pp.request(d.sim(), requestFor(TargetKind::Body, w.planet, 20000.0, 50));
        CHECK(second == first + 1);
        CHECK(pp.latestRequest() == second);
        REQUIRE(pp.waitIdle(60.0));                                      // the first job was dropped at its next chunk, not run out
        REQUIRE(pp.poll());
        const PredictedPath& r = pp.result();
        CHECK(r.requestId == second);
        CHECK(r.id == w.planet);
        CHECK(r.count == 51);
        // A target that does not exist queues nothing.
        CHECK(pp.request(d.sim(), requestFor(TargetKind::Body, 12345, 100.0, 10)) == 0);
        CHECK(pp.request(d.sim(), requestFor(TargetKind::Shuttle, w.moon, 100.0, 10)) == 0);
        CHECK(pp.request(d.sim(), requestFor(TargetKind::None, 0, 100.0, 10)) == 0);
        // cancel() drops the result.
        pp.cancel();
        CHECK_FALSE(pp.result().valid());
        CHECK_FALSE(pp.busy());
    }

    TEST_CASE("the step budget truncates a path, a hit shuttle ends it")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        PredictRequest rq = requestFor(TargetKind::Body, w.moon, 20.0 * w.moonPeriod, 400);
        rq.maxBodySteps = 50;
        const PredictedPath t = predictNow(d.sim(), rq);
        CHECK(t.truncated);
        CHECK(t.count >= 1);
        CHECK(t.count < 401);

        // A shuttle dropped onto a planet at rest: the path stops at the contact point on the surface.
        BodyDesc planet;
        planet.mu = kPlanetMu;
        planet.radius = 6.371e6;
        ResolvedScene res = sim_test::sceneOf({planet});
        res.shuttles.push_back(sim_test::resolvedShuttle(900, {2.0e7, 0, 0}, {0, 0, 0}));
        SimDriver e(res, SceneSettings{});
        const PredictedPath h = predictNow(e.sim(), requestFor(TargetKind::Shuttle, 900, 6000.0, 60));
        CHECK(h.targetEnded);
        CHECK(h.count < 61);
        REQUIRE(h.count >= 2);
        const Vec3 last = h.target[h.count - 1];
        const Vec3 pe{0, 0, 0};
        CHECK(std::hypot(last.x - pe.x, last.y - pe.y) == doctest::Approx(6.371e6 + 2.0).epsilon(1e-6));   // on the surface (planet + shuttle radius)
    }

    TEST_CASE("the builder requests, receives and draws the path of the followed shuttle")
    {
        ResolvedScene res = driver_test::lucernaWithShuttles(2);
        SimDriver d(res, SceneSettings{});
        d.advanceTo(30.0);
        OverlayBuilder ob;
        ob.settings.predictedPath = true;
        ob.settings.predictSeconds = 2000.0;
        ob.settings.predictSamples = 100;
        OverlayView v = viewAt({0, 0, 0}, 1e4);
        v.follow = {TargetKind::Shuttle, 901};
        RenderFrame f = frameOf();
        ob.build(d.state(), d.sim(), 1, v, f);
        CHECK(f.underlay.empty());                           // the worker has not delivered yet: the frame is not delayed
        CHECK(ob.stats().predictionPending);
        REQUIRE(ob.predictor().waitIdle(60.0));
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, v, f);
        REQUIRE(ob.stats().predictionDrawn);
        REQUIRE(f.underlay.strips.size() == 1);
        const LineStrip& s = f.underlay.strips[0];
        CHECK(s.count >= 101);                               // the shuttle's own position and the samples after now
        CHECK(s.count <= 102);
        const DisplayShuttle* sh = nullptr;
        for (const DisplayShuttle& x : d.state().shuttles)
            if (x.id == 901) sh = &x;
        REQUIRE(sh != nullptr);
        CHECK(std::abs(f.underlay.vertices[s.first].x - static_cast<float>(sh->r.x)) < 1e5f);   // starts at the shuttle
        // Without a target (not following, none set) the path goes away and the worker is idle.
        v.follow = {};
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, v, f);
        CHECK(f.underlay.empty());
        CHECK_FALSE(ob.predictor().busy());
        // Explicit target beats the camera.
        ob.setPredictionTarget(TargetKind::Shuttle, 900);
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, v, f);
        CHECK(ob.stats().predictionPending);
    }

    TEST_CASE("the predicted path is re-expressed in the follow-body frame")
    {
        SunPlanetMoon w;
        SimDriver d(w.res, SceneSettings{});
        d.advanceTo(1000.0);
        OverlayBuilder ob;
        ob.settings.predictedPath = true;
        ob.settings.predictSeconds = w.moonPeriod;
        ob.settings.predictSamples = 120;
        const DisplayBody& planet = *bodyOf(d.state(), w.planet);
        OverlayView v = viewAt(planet.r, 1e6);
        v.frameMode = FrameMode::FollowBody;
        v.frameBody = w.planet;
        v.follow = {TargetKind::Body, w.moon};
        RenderFrame f = frameOf();
        ob.build(d.state(), d.sim(), 1, v, f);
        REQUIRE(ob.predictor().waitIdle(60.0));
        f.clearObjects();
        ob.build(d.state(), d.sim(), 1, v, f);
        REQUIRE(ob.stats().predictionDrawn);
        const LineStrip& s = f.underlay.strips[0];
        // Seen from the planet, one period of the moon is a closed loop of radius ~ kMoonA about the camera.
        double rmin = 1e300, rmax = 0.0, sweep = 0.0, prev = 0.0;
        for (std::uint32_t k = 0; k < s.count; ++k) {
            const LineVertex& vx = f.underlay.vertices[s.first + k];
            const double r = std::hypot(vx.x, vx.y), a = std::atan2(vx.y, vx.x);
            rmin = std::min(rmin, r);
            rmax = std::max(rmax, r);
            if (k > 0) {
                double da = a - prev;
                if (da > kPi) da -= 2.0 * kPi;
                if (da < -kPi) da += 2.0 * kPi;
                sweep += da;
            }
            prev = a;
        }
        CHECK(rmin > 0.9 * kMoonA);
        CHECK(rmax < 1.1 * kMoonA);
        CHECK(sweep / (2.0 * kPi) > 0.95);
    }
}
