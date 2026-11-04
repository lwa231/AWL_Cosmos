#include <doctest.h>
#include <cmath>
#include <set>
#include "app/explosions.h"
#include "test_app_driver_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace driver_test;

// Explosions (plan 9.7, 6.5, 8 item 3): registry, double-precision anchors, spawn parameters, sim-time lifetimes with a minimum
// real-time visibility, oldest-first recycling and the per-frame data for the GPU. GL-free.

namespace {

constexpr double kMuDomus = 3.986e14, kRDomus = 6.371e6;

DisplayState stateWithDomus(double t, const Vec3& r, const Vec3& v)
{
    DisplayState st;
    st.t = t;
    DisplayBody b;
    b.id = 7;
    b.name = "Domus";
    b.mu = kMuDomus;
    b.radius = kRDomus;
    b.spinRate = 7.0e-5;
    b.atmosphere = Atmosphere{1.2, 7000.0};
    b.r = r;
    b.v = v;
    st.bodies.push_back(b);
    return st;
}

ExplosionSpec specOn(const DisplayBody& b, const Vec3& dir, std::uint64_t source, double time = 100.0, double impactorRadius = 2.0)
{
    ExplosionSpec s;
    s.contact = b.r + dir * (b.radius + 2.0);
    s.refBody = b.id;
    s.refPos = b.r;
    s.refVel = b.v;
    s.refSpin = b.spinRate;
    s.normal = dir;
    s.impactVelocity = dir * -3000.0;
    s.impactorRadius = impactorRadius;
    s.sourceId = source;
    s.time = time;
    return s;
}

const ExplosionSystem::ToCamera kToCamera = [](const Vec3& w, float& x, float& y) {
    x = static_cast<float>(w.x);
    y = static_cast<float>(w.y);
};

} // namespace

TEST_SUITE("app explosions")
{
    TEST_CASE("spawn parameters grow with the impactor and are capped")
    {
        const ExplosionParams a = explosionParams(2.0, 3000.0);
        const ExplosionParams b = explosionParams(2.0e3, 3000.0);
        const ExplosionParams c = explosionParams(1.0e9, 1.0e5);
        CHECK(a.fragments > 1000);
        CHECK(b.fragments > a.fragments);
        CHECK(c.fragments <= 12000);
        CHECK(c.fragments >= b.fragments);
        CHECK(b.lifeMax > a.lifeMax);
        CHECK(a.speedMax > a.speedMin);
        CHECK(a.speedMin > 0.0);
        CHECK(explosionParams(2.0, 6000.0).speedMax > explosionParams(2.0, 10.0).speedMax);
        CHECK(std::isfinite(explosionParams(0.0, std::nan("")).speedMax));
    }

    TEST_CASE("a spawn is repeatable and uses its own random stream")
    {
        const DisplayState st = stateWithDomus(100.0, {1.0e11, 2.0e11, 0.0}, {0.0, 3.0e4, 0.0});
        ExplosionSystem a, b, c;
        const std::uint32_t sa = a.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
        const std::uint32_t sb = b.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
        c.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 901));
        CHECK(sa != 0);
        CHECK(sa == sb);
        ExplosionFrame fa, fb, fc;
        a.update(st, 0.0, 0.0, kToCamera, fa);
        b.update(st, 0.0, 0.0, kToCamera, fb);
        c.update(st, 0.0, 0.0, kToCamera, fc);
        REQUIRE(fa.spawns.size() == 1);
        REQUIRE(fb.spawns.size() == 1);
        REQUIRE(fc.spawns.size() == 1);
        REQUIRE(fa.spawns[0].fragments.size() == fb.spawns[0].fragments.size());
        bool same = true, differs = false;
        for (std::size_t i = 0; i < fa.spawns[0].fragments.size(); ++i) {
            const FragmentGpu &x = fa.spawns[0].fragments[i], &y = fb.spawns[0].fragments[i], &z = fc.spawns[0].fragments[i];
            if (x.px != y.px || x.vx != y.vx || x.vy != y.vy || x.life != y.life || x.seed != y.seed) same = false;
            if (x.vx != z.vx || x.seed != z.seed) differs = true;
        }
        CHECK(same);
        CHECK(differs);
    }

    TEST_CASE("fragments start near the anchor, inside the cloud limits, owned by the explosion")
    {
        const DisplayState st = stateWithDomus(100.0, {1.0e11, 2.0e11, 0.0}, {0.0, 3.0e4, 0.0});
        ExplosionSystem sys;
        const ExplosionSpec spec = specOn(st.bodies[0], {0.0, 1.0, 0.0}, 900);
        const std::uint32_t serial = sys.spawn(spec);
        const ExplosionParams p = explosionParams(spec.impactorRadius, 3000.0);
        ExplosionFrame f;
        sys.update(st, 0.0, 0.0, kToCamera, f);
        REQUIRE(f.active());
        REQUIRE(f.spawns.size() == 1);
        CHECK(f.fragmentCount == f.spawns[0].fragments.size());
        CHECK(f.table.size() == kExplosionMaxLive);
        const double ground = 7.0e-5 * (kRDomus + 2.0);   // the surface moves with the spin: ~446 m/s along -x at the top
        for (const FragmentGpu& g : f.spawns[0].fragments) {
            CHECK(g.serial == serial);
            CHECK(std::fabs(g.px) < 10.0);
            CHECK(g.py > 0.0);   // above the surface at the contact point
            CHECK(g.py < 10.0);
            // the speed relative to the moving ground stays inside the cloud limits (plus the skid, which is zero for a head-on impact)
            const double vx = g.vx + ground, vy = g.vy;
            CHECK(std::sqrt(vx * vx + vy * vy) <= p.speedMax * 1.0001);
            CHECK(vy > -1.0);   // thrown outward
            CHECK(g.life >= p.lifeMin * 0.999);
            CHECK(g.life <= p.lifeMax * 1.001);
            CHECK(g.ballistic >= p.ballisticMin * 0.999);
            CHECK(g.ballistic <= p.ballisticMax * 1.001);
            CHECK(g.minReal >= 2.0f);
        }
    }

    TEST_CASE("the anchor is a double offset from its body and follows it")
    {
        // Far from the origin: 1.5e11 m. In float the contact point would be good to ~16 kilometres; the double offset keeps metres.
        DisplayState st = stateWithDomus(100.0, {1.5e11, 0.0, 0.0}, {0.0, 3.0e4, 0.0});
        ExplosionSystem sys;
        ExplosionSpec spec = specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900);
        spec.contact.x += 0.3;   // sub-metre detail that a float world position would lose
        sys.spawn(spec);
        ExplosionFrame f;
        sys.update(st, 0.0, 0.0, kToCamera, f);
        REQUIRE(sys.explosions().size() == 1);
        CHECK(sys.explosions()[0].anchor.x == spec.contact.x);
        CHECK(sys.explosions()[0].offset.x == doctest::Approx(kRDomus + 2.3).epsilon(1e-9));

        // a second later the body has moved 3e4 m along y; the anchor moved with it, to the metre
        st.t = 101.0;
        st.bodies[0].r = {1.5e11 - 12.0, 3.0e4, 0.0};
        Vec3 seenWorld;
        sys.update(st, 1.0, 1.0 / 60.0, [&](const Vec3& w, float& x, float& y) {
            seenWorld = w;
            x = static_cast<float>(w.x - 1.5e11);   // the app subtracts the camera in double
            y = static_cast<float>(w.y);
        }, f);
        CHECK(seenWorld.x == doctest::Approx(1.5e11 - 12.0 + kRDomus + 2.3).epsilon(1e-15));
        CHECK(seenWorld.y == doctest::Approx(3.0e4).epsilon(1e-12));
        REQUIRE(f.active());
        CHECK(f.table[sys.explosions()[0].slot].anchorX == doctest::Approx(static_cast<float>(kRDomus + 2.3 - 12.0)).epsilon(1e-6));
    }

    TEST_CASE("the table carries the nearby bodies relative to the anchor and the frame's time step")
    {
        DisplayState st = stateWithDomus(100.0, {1.0e9, 0.0, 0.0}, {0.0, 1.0e3, 0.0});
        // a close, heavy moon (2.5% of Domus's pull at the anchor) and a star far away (far below 1%)
        DisplayBody moon;
        moon.id = 8;
        moon.name = "Luna";
        moon.mu = 4.9e13;
        moon.radius = 1.7e6;
        moon.r = st.bodies[0].r + Vec3{1.0e7, 1.0e7, 0.0};
        moon.v = st.bodies[0].v + Vec3{-1000.0, 0.0, 0.0};
        DisplayBody star;
        star.id = 9;
        star.name = "Sol";
        star.mu = 1.3e20;
        star.radius = 7.0e8;
        star.r = {-1.5e11, 0.0, 0.0};
        st.bodies.push_back(moon);
        st.bodies.push_back(star);
        ExplosionSystem sys;
        sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
        ExplosionFrame f;
        sys.update(st, 5.0, 0.016, kToCamera, f);
        const ExplosionGpu& g = f.table[sys.explosions()[0].slot];
        CHECK(g.serial == sys.explosions()[0].serial);
        CHECK(g.dtSim == 0.0f);   // the frame of the spawn does not age the cloud
        REQUIRE(g.bodyCount >= 1);
        CHECK(g.bodies[0].mu == doctest::Approx(static_cast<float>(kMuDomus)));   // the anchor's body first
        CHECK(g.bodies[0].x == doctest::Approx(-(kRDomus + 2.0)).epsilon(1e-6));  // its centre relative to the anchor
        CHECK(g.bodies[0].altitude == doctest::Approx(2.0).epsilon(1e-3));
        CHECK(g.bodies[0].rho0 == doctest::Approx(1.2f));
        CHECK(g.bodies[0].spin == doctest::Approx(7.0e-5f));
        // later frames carry the time step
        sys.update(st, 5.0, 0.016, kToCamera, f);
        const ExplosionGpu& h = f.table[sys.explosions()[0].slot];
        CHECK(h.dtSim == doctest::Approx(5.0f));
        CHECK(h.dtReal == doctest::Approx(0.016f));
        std::set<float> mus;
        for (std::uint32_t k = 0; k < h.bodyCount; ++k) mus.insert(h.bodies[k].mu);
        CHECK(mus.count(static_cast<float>(4.9e13)) == 1);   // the moon is listed ...
        CHECK(mus.count(static_cast<float>(1.3e20)) == 0);   // ... the far star is not
        for (std::uint32_t k = 1; k < h.bodyCount; ++k) CHECK(h.bodies[k].indirectX != 0.0f);   // tidal term for the others
        CHECK(h.bodies[0].indirectX == 0.0f);
    }

    TEST_CASE("lifetimes run in sim time with a minimum real-time visibility")
    {
        const DisplayState st = stateWithDomus(100.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        const double life = explosionParams(2.0, 3000.0).lifeMax;
        ExplosionFrame f;

        {   // high warp: 1e5x at 60 fps = 1666 sim s per frame. Far beyond the sim lifetime after one frame, yet visible for ~2 real s
            ExplosionSystem sys;
            sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
            sys.update(st, 0.0, 0.0, kToCamera, f);
            double real = 0.0;
            bool aliveAt1 = false, aliveAt19 = false;
            for (int i = 0; i < 400 && sys.liveExplosions() > 0; ++i) {
                sys.update(st, 1666.0, 1.0 / 60.0, kToCamera, f);
                real += 1.0 / 60.0;
                if (std::fabs(real - 1.0) < 0.01 && sys.liveExplosions() > 0) aliveAt1 = true;
                if (std::fabs(real - 1.9) < 0.01 && sys.liveExplosions() > 0) aliveAt19 = true;
            }
            CHECK(aliveAt1);
            CHECK(aliveAt19);
            CHECK(sys.liveExplosions() == 0);
            CHECK(real >= 2.0);
            CHECK(real < 3.2);   // gone after at most 1.4 x the minimum visibility
            CHECK_FALSE(f.active());
        }
        {   // 1x: the sim lifetime governs (longer than the real minimum)
            ExplosionSystem sys;
            sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
            sys.update(st, 0.0, 0.0, kToCamera, f);
            double sim = 0.0;
            while (sys.liveExplosions() > 0 && sim < 200.0) {
                sys.update(st, 1.0 / 60.0, 1.0 / 60.0, kToCamera, f);
                sim += 1.0 / 60.0;
            }
            CHECK(sim == doctest::Approx(life).epsilon(0.02));
        }
        {   // a stopped clock stands still
            ExplosionSystem sys;
            sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
            for (int i = 0; i < 1000; ++i) sys.update(st, 0.0, 1.0 / 60.0, kToCamera, f);
            CHECK(sys.liveExplosions() == 1);
            CHECK(sys.explosions()[0].simAge == 0.0);
            CHECK(sys.explosions()[0].realAge == 0.0);
        }
        {   // a stall counts as at most maxRealDt of visibility
            ExplosionSystem sys;
            sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 900));
            sys.update(st, 0.0, 0.0, kToCamera, f);
            sys.update(st, 1.0e5, 30.0, kToCamera, f);
            CHECK(sys.liveExplosions() == 1);
        }
    }

    TEST_CASE("fragments are recycled oldest-first within the pool")
    {
        const DisplayState st = stateWithDomus(100.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        ExplosionConfig cfg;
        cfg.maxFragments = 6000;   // room for two shuttle clouds, not three
        ExplosionSystem sys(cfg);
        const std::uint32_t s1 = sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 1));
        const std::uint32_t s2 = sys.spawn(specOn(st.bodies[0], {0.0, 1.0, 0.0}, 2));
        REQUIRE(sys.liveExplosions() == 2);
        const std::uint32_t before = sys.liveFragments();
        CHECK(before <= cfg.maxFragments);
        const std::uint32_t s3 = sys.spawn(specOn(st.bodies[0], {-1.0, 0.0, 0.0}, 3));
        CHECK(sys.liveFragments() <= cfg.maxFragments);
        std::uint32_t live1 = 0, live2 = 0, live3 = 0;
        for (const ExplosionSystem::Explosion& e : sys.explosions()) {
            if (e.serial == s1) live1 = e.live;
            if (e.serial == s2) live2 = e.live;
            if (e.serial == s3) live3 = e.live;
        }
        CHECK(live3 > 0);
        CHECK(live1 < live2);   // the oldest lost slots first
        // the three clouds together never own more than the pool and no slot is shared
        std::set<std::uint32_t> slots;
        for (const ExplosionSystem::Explosion& e : sys.explosions())
            for (const ExplosionSystem::Range& r : e.ranges)
                for (std::uint32_t i = 0; i < r.count; ++i) CHECK(slots.insert(r.first + i).second);
        CHECK(sys.fragmentHighWater() <= cfg.maxFragments);

        // the budget of the plan: 20k fragments, 8 sparks each = 160k
        CHECK(kExplosionMaxFragments == 20000);
        CHECK(kExplosionMaxFragments * kExplosionSparksPerFragment == 160000);
    }

    TEST_CASE("at most kExplosionMaxLive explosions at once; the oldest makes room")
    {
        const DisplayState st = stateWithDomus(100.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        ExplosionSystem sys;
        std::uint32_t first = 0;
        for (std::uint32_t i = 0; i < kExplosionMaxLive + 10; ++i) {
            const std::uint32_t s = sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 100 + i, 100.0 + i, 0.2));
            if (i == 0) first = s;
        }
        CHECK(sys.liveExplosions() <= kExplosionMaxLive);
        CHECK(sys.liveFragments() <= kExplosionMaxFragments);
        for (const ExplosionSystem::Explosion& e : sys.explosions()) CHECK(e.serial != first);
        std::set<std::uint32_t> slots;
        for (const ExplosionSystem::Explosion& e : sys.explosions()) CHECK(slots.insert(e.slot).second);
    }

    TEST_CASE("a shuttle that hits Domus makes an explosion at its contact point")
    {
        scene_test::Lucerna L;
        const std::size_t domus = L.storeIndex("Domus");
        ResolvedScene res = std::move(L.res);
        const BodyId domusId = res.bodies.ids[domus];
        const double RD = res.bodies.radius[domus];
        Vec3 pos, vel;
        sim_test::circularOrbit(res.bodies.mu[domus], RD + 1.5e5, 1.0, sim_test::bodyPos(res.bodies, domus),
                                sim_test::bodyVel(res.bodies, domus), pos, vel);
        res.shuttles.push_back(sim_test::resolvedShuttle(900, pos, vel, "Retrograde"));

        SimulationOptions opt;
        opt.ias15.dtMax = 120.0;
        DriverOptions dopt;
        dopt.physicsBudget_s = 0.0;
        SimDriver d(res, SceneSettings{}, opt, dopt);
        d.warp().set(8.0);
        ExplosionSystem sys;
        ExplosionFrame f;
        bool spawned = false;
        for (int frame = 0; frame < 8000 && !spawned; ++frame) {
            d.advance(0.125);
            sys.handleEvents(d.dueEvents(), d.state(), d.sim());
            if (sys.liveExplosions() > 0) spawned = true;
        }
        REQUIRE(spawned);
        REQUIRE(sys.liveExplosions() == 1);
        sys.update(d.state(), 1.0, 0.125, kToCamera, f);
        const ExplosionSystem::Explosion& ex = sys.explosions()[0];
        CHECK(ex.refBody == domusId);
        const DisplayBody* dom = nullptr;
        for (const DisplayBody& b : d.state().bodies)
            if (b.id == domusId) dom = &b;
        REQUIRE(dom != nullptr);
        const double alt = norm(ex.anchor - dom->r) - RD;
        MESSAGE("anchor altitude " << alt << " m");
        CHECK(std::fabs(alt) < 50.0);   // the contact point is on the surface (a few metres of shuttle radius and 1 s of drift)
        CHECK(f.active());
        CHECK(f.fragmentCount > 1000);
    }

    TEST_CASE("a body collision explodes on the survivor, scaled by the absorbed body")
    {
        DisplayState st = stateWithDomus(100.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        DisplayBody rock;
        rock.id = 11;
        rock.name = "Rock";
        rock.mu = 1.0e6;
        rock.radius = 5.0e3;
        rock.r = {kRDomus + 5.0e3, 0.0, 0.0};
        rock.v = {-9000.0, 0.0, 0.0};
        st.bodies.push_back(rock);

        const Simulation emptySim(ResolvedScene{}, SceneSettings{});
        ExplosionSystem sys;
        ExplosionFrame f;
        sys.update(st, 0.0, 0.0, kToCamera, f);   // lets the system see the bodies (their radius and speed)
        std::vector<Event> due;
        due.push_back(makeCollision(100.0, 7, 11, 9000.0, {kRDomus, 0.0, 0.0}));
        due.push_back(makeMerge(100.0, 7, 11));
        sys.handleEvents(due, st, emptySim);
        // The simulation argument is only consulted for the dense-output position; with an empty one the display state is used.
        REQUIRE(sys.liveExplosions() == 1);
        CHECK(sys.explosions()[0].refBody == 7);
        CHECK(sys.liveFragments() > explosionParams(2.0, 9000.0).fragments);   // a 5 km rock beats a 2 m shuttle
        sys.update(st, 1.0, 1.0 / 60.0, kToCamera, f);
        CHECK(norm(sys.explosions()[0].anchor - Vec3{kRDomus, 0.0, 0.0}) < 1.0e-6);

        // other events spawn nothing
        const std::size_t before = sys.liveExplosions();
        std::vector<Event> other;
        other.push_back(makeAtmosphereEntry(101.0, 900, 7));
        other.push_back(makeFuelEmpty(101.0, 900));
        other.push_back(makeMerge(101.0, 7, 11));
        sys.handleEvents(other, st, emptySim);
        CHECK(sys.liveExplosions() == before);
    }

    TEST_CASE("clear forgets every explosion")
    {
        const DisplayState st = stateWithDomus(100.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        ExplosionSystem sys;
        sys.spawn(specOn(st.bodies[0], {1.0, 0.0, 0.0}, 1));
        sys.clear();
        ExplosionFrame f;
        sys.update(st, 0.0, 0.0, kToCamera, f);
        CHECK(sys.liveExplosions() == 0);
        CHECK_FALSE(f.active());
        CHECK(f.spawns.empty());
    }
}
