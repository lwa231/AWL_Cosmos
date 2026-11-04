#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <thread>
#include "core/gravity.h"
#include "core/ias15.h"
#include "test_scene_util.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Shuttles against IAS15 dense output (plan 6.3, 6.4), atmospheres in orbit (5.5, 5.6) and thread independence (4.5, 4.6).

namespace {
// The first n bodies of an IAS15 system (the rest are test particles that the shuttle logic must not see).
class PrefixEphemeris final : public IBodyEphemeris {
public:
    PrefixEphemeris(const IAS15& ias, std::size_t n) : ias_(ias), n_(n) {}
    std::size_t count() const override { return n_; }
    Vec3 position(std::size_t i, double t) const override { return ias_.positionOf(i, ias_.tauAt(t)); }
    Vec3 velocity(std::size_t i, double t) const override { return ias_.velocityOf(i, ias_.tauAt(t)); }

private:
    const IAS15& ias_;
    std::size_t n_;
};
} // namespace

TEST_SUITE("shuttles with IAS15 bodies")
{
    TEST_CASE("a shuttle on the dense output of the Lucerna bodies tracks a massless particle integrated inside IAS15")
    {
        // Reference: the shuttle as an extra body with mu = 0 in the 14-body IAS15 run (15th order). Shuttle: DOPRI5 against the
        // dense output of the bodies' last step, step after step. 300 km above Domus, 4 orbits (about 3 h).
        scene_test::Lucerna L;
        const Bodies& B = L.res.bodies;
        const std::size_t n = B.size();
        const std::size_t domus = L.storeIndex("Domus");

        BodyState s = B.state;
        std::vector<double> mu = B.mu;
        const double rOrb = B.radius[domus] + 3.0e5;
        const Vec3 pD{s.x[domus], s.y[domus], s.z[domus]}, vD{s.vx[domus], s.vy[domus], s.vz[domus]};
        const double vc = std::sqrt(B.mu[domus] / rOrb);
        const Vec3 r0 = pD + Vec3{rOrb, 0.0, 0.0}, v0 = vD + Vec3{0.0, vc, 0.0};
        for (std::vector<double>* a : {&s.x, &s.y, &s.z, &s.vx, &s.vy, &s.vz}) a->push_back(0.0);
        s.x[n] = r0.x; s.y[n] = r0.y; s.z[n] = r0.z;
        s.vx[n] = v0.x; s.vy[n] = v0.y; s.vz[n] = v0.z;
        mu.push_back(0.0);

        ControllerSet cs;
        Shuttles sh;
        REQUIRE(sh.add(shuttleDesc(1, r0, v0)));
        IAS15 ias;
        const ForceFn force = [&mu](const BodyState& st, Accel& a) { computeAccelerations(st, mu, a); };
        const PrefixEphemeris eph(ias, n);
        const ShuttleEnvironment env(B, eph, cs);

        const double T = 4.0 * 2.0 * PI * std::sqrt(rOrb * rOrb * rOrb / B.mu[domus]);
        double t = 0.0;
        std::vector<Event> ev;
        ShuttleStats st;
        long bodySteps = 0;
        while (t < T) {
            const double tPrev = t;
            ias.step(s, force, t, T);
            REQUIRE_FALSE(ias.lastFailed());
            REQUIRE(ias.hasDenseOutput());
            propagateShuttle(sh, 0, tPrev, t, env, ev, nullptr, &st);
            ++bodySteps;
        }
        const Vec3 rS = shuttlePos(sh, 0), rRef{s.x[n], s.y[n], s.z[n]};
        const Vec3 vS = shuttleVel(sh, 0), vRef{s.vx[n], s.vy[n], s.vz[n]};
        // Measured: 3.3e-8 of the orbit radius (position) and of the circular speed (velocity) after 4 orbits, 105 body steps
        CHECK(norm(rS - rRef) / rOrb < 2e-7);
        CHECK(norm(vS - vRef) / vc < 2e-7);
        CHECK(bodySteps > 50);
        CHECK(st.acceptedSteps > bodySteps);
        CHECK(ev.empty());
    }

    TEST_CASE("results do not depend on the order of shuttles or on the thread that runs them (bit-identical)")
    {
        scene_test::Lucerna L;
        const Bodies& B = L.res.bodies;
        const std::size_t n = B.size();
        const std::size_t domus = L.storeIndex("Domus");
        const Vec3 pD{B.state.x[domus], B.state.y[domus], B.state.z[domus]};
        const Vec3 vD{B.state.vx[domus], B.state.vy[domus], B.state.vz[domus]};

        // 24 shuttles with all four controllers, at different altitudes, some of them dipping into the atmosphere
        auto fill = [&](Shuttles& s) {
            const ControllerKind kinds[] = {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde, ControllerKind::Wander};
            for (ShuttleId id = 1; id <= 24; ++id) {
                const double alt = 6.0e4 + 2.5e4 * static_cast<double>(id % 8);
                const double r = B.radius[domus] + alt;
                const double phi = 0.3 * static_cast<double>(id);
                const double ecc = id % 3 == 0 ? 0.9 : 1.0;
                const double vc = std::sqrt(B.mu[domus] / r) * ecc;
                ShuttleDesc d = shuttleDesc(id, pD + Vec3{r * std::cos(phi), r * std::sin(phi), 0.0},
                                            vD + Vec3{-vc * std::sin(phi), vc * std::cos(phi), 0.0}, kinds[id % 4]);
                d.heading = phi + PI / 2.0;
                REQUIRE(s.add(d));
            }
        };
        ControllerSet cs;
        auto runAll = [&](Shuttles& s, int mode) {
            IAS15 ias;
            BodyState st = B.state;
            const ForceFn force = [&B](const BodyState& x, Accel& a) { computeAccelerations(x, B.mu, a); };
            const Ias15Ephemeris eph(ias);
            const ShuttleEnvironment env(B, eph, cs);
            double t = 0.0;
            std::vector<std::vector<Event>> bufs(4);
            for (int step = 0; step < 5; ++step) {
                const double tPrev = t;
                ias.step(st, force, t);
                const std::size_t N = s.size();
                if (mode == 0) { // forward, one thread
                    for (std::size_t i = 0; i < N; ++i) propagateShuttle(s, i, tPrev, t, env, bufs[0]);
                } else if (mode == 1) { // reverse
                    for (std::size_t i = N; i-- > 0;) propagateShuttle(s, i, tPrev, t, env, bufs[0]);
                } else { // four threads, fixed partitions, per-thread buffers
                    std::vector<std::thread> pool;
                    for (std::size_t w = 0; w < 4; ++w)
                        pool.emplace_back([&, w] {
                            for (std::size_t i = w * N / 4; i < (w + 1) * N / 4; ++i) propagateShuttle(s, i, tPrev, t, env, bufs[w]);
                        });
                    for (std::thread& th : pool) th.join();
                }
            }
            return EventQueue::merge(bufs);
        };

        Shuttles a, b, c;
        fill(a);
        fill(b);
        fill(c);
        const EventQueue ea = runAll(a, 0);
        const EventQueue eb = runAll(b, 1);
        const EventQueue ec = runAll(c, 2);
        REQUIRE(a.size() == 24u);
        bool anyMoved = false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            CAPTURE(i);
            CHECK(sameBits(a.state.x[i], b.state.x[i]));
            CHECK(sameBits(a.state.y[i], b.state.y[i]));
            CHECK(sameBits(a.state.vx[i], b.state.vx[i]));
            CHECK(sameBits(a.state.vy[i], b.state.vy[i]));
            CHECK(sameBits(a.heading[i], b.heading[i]));
            CHECK(sameBits(a.spin[i], b.spin[i]));
            CHECK(sameBits(a.fuelMass[i], b.fuelMass[i]));
            CHECK(sameBits(a.state.x[i], c.state.x[i]));
            CHECK(sameBits(a.state.y[i], c.state.y[i]));
            CHECK(sameBits(a.state.vx[i], c.state.vx[i]));
            CHECK(sameBits(a.state.vy[i], c.state.vy[i]));
            CHECK(sameBits(a.heading[i], c.heading[i]));
            CHECK(sameBits(a.fuelMass[i], c.fuelMass[i]));
            CHECK(a.alive[i] == c.alive[i]);
            if (a.fuelMass[i] < 3000.0) anyMoved = true;
        }
        CHECK(anyMoved); // the active controllers really burned
        REQUIRE(ea.size() == ec.size());
        REQUIRE(ea.size() == eb.size());
        for (std::size_t k = 0; k < ea.size(); ++k) {
            CHECK(ea.events()[k].t == ec.events()[k].t);
            CHECK(ea.events()[k].id == ec.events()[k].id);
            CHECK(ea.events()[k].type == ec.events()[k].type);
        }
        (void)n;
    }
}

TEST_SUITE("shuttles in an atmosphere")
{
    // A Domus-like planet with its atmosphere (rho0 1.22, H 7000), static, optionally spinning (5.8 h period)
    struct DomusWorld {
        Bodies b;
        LinearEphemeris eph;
        ControllerSet cs;
        ShuttleEnvironment env;
        explicit DomusWorld(double spin) : b(make(spin)), eph(b), env(b, eph, cs) {}
        static Bodies make(double spin)
        {
            Bodies bodies;
            BodyDesc d = bodyDesc(kMuDomus, kRDomus);
            d.atmosphere = Atmosphere{1.22, 7000.0};
            d.spinRate = spin;
            bodies.add(d);
            return bodies;
        }
    };

    // Periapsis 70 km above the surface, apoapsis radius 1.4e6 m. direction +1 = prograde (counter-clockwise).
    Shuttles decayingOrbit(double direction)
    {
        const double rp = kRDomus + 7.0e4, ra = 1.4e6;
        const double a = 0.5 * (rp + ra);
        const double vp = std::sqrt(kMuDomus * (2.0 / rp - 1.0 / a));
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {rp, 0.0, 0.0}, {0.0, direction * vp, 0.0})));
        return s;
    }

    double orbitEnergy(const Shuttles& s) { return 0.5 * norm2(shuttleVel(s, 0)) - kMuDomus / norm(shuttlePos(s, 0)); }

    TEST_CASE("an orbit grazing the atmosphere loses energy only there; none is lost above it")
    {
        DomusWorld W(0.0);
        // high orbit: periapsis well above the top (98 km): energy conserved by the gravity step alone
        const double rp = kRDomus + 3.0e5, ra = 1.4e6, a = 0.5 * (rp + ra);
        const double vp = std::sqrt(kMuDomus * (2.0 / rp - 1.0 / a));
        Shuttles high;
        REQUIRE(high.add(shuttleDesc(1, {rp, 0.0, 0.0}, {0.0, vp, 0.0})));
        const double e0 = orbitEnergy(high);
        std::vector<Event> ev;
        propagateShuttle(high, 0, 0.0, 3.0 * keplerPeriod(kMuDomus, a), W.env, ev);
        CHECK(std::fabs(orbitEnergy(high) - e0) / std::fabs(e0) < 1e-6);

        // low orbit: periapsis at 70 km altitude dips into the air
        Shuttles low = decayingOrbit(+1.0);
        const double eLow0 = orbitEnergy(low);
        const double aLow = 0.5 * (kRDomus + 7.0e4 + 1.4e6);
        propagateShuttle(low, 0, 0.0, 1.0 * keplerPeriod(kMuDomus, aLow), W.env, ev);
        CHECK(orbitEnergy(low) < eLow0 - 1.0e4);          // the 70 km periapsis costs tens of kJ/kg per orbit (70.8 kJ/kg in two)
        CHECK(low.fuelMass[0] == 3000.0);                  // drag does not touch the fuel
    }

    TEST_CASE("drag converges with the step: the density-profile limit makes the decayed orbit independent of the step")
    {
        // Energy lost in two orbits with periapsis at 70 km altitude. Reference: 16x finer than the default limit.
        auto lossFor = [](double resolution) {
            DomusWorld W(0.0);
            W.env.atmosphereResolution = resolution;
            W.env.activeStepFraction = 0.02;
            Shuttles s = decayingOrbit(+1.0);
            const double e0 = orbitEnergy(s);
            std::vector<Event> ev;
            propagateShuttle(s, 0, 0.0, 2.0 * keplerPeriod(kMuDomus, 0.5 * (kRDomus + 7.0e4 + 1.4e6)), W.env, ev);
            return e0 - orbitEnergy(s);
        };
        const double ref = lossFor(0.1 / 16.0);
        CHECK(ref > 5.0e4); // 70.8 kJ/kg
        const double atDefault = lossFor(0.1);
        const double coarse = lossFor(0.8);
        const double off = lossFor(0.0); // only the 0.02 dynamical-time limit (5.9 s = 17 km of travel at periapsis)
        // Measured relative deviations from the reference: 0.1 -> 3.7e-6, 0.8 -> 1.2e-5, off -> 3.7e-4 (the trapezoid rule of the
        // split drag sampling the 7 km scale height with 17 km steps)
        CHECK(std::fabs(atDefault - ref) / ref < 2e-5);
        CHECK(std::fabs(coarse - ref) / ref < 1e-4);
        CHECK(std::fabs(off - ref) / ref > 10.0 * std::fabs(atDefault - ref) / ref);
        CHECK(std::fabs(off - ref) / ref < 5e-3);
    }

    TEST_CASE("drag is relative to the rotating air: a retrograde orbit decays faster than the prograde one")
    {
        const double spin = 2.0 * PI / (5.8 * 3600.0); // Domus rotates counter-clockwise
        const double T = 2.0 * keplerPeriod(kMuDomus, 0.5 * (kRDomus + 7.0e4 + 1.4e6));
        auto loss = [&](double dir, double bodySpin) {
            DomusWorld W(bodySpin);
            Shuttles s = decayingOrbit(dir);
            const double e0 = orbitEnergy(s);
            std::vector<Event> ev;
            propagateShuttle(s, 0, 0.0, T, W.env, ev);
            return e0 - orbitEnergy(s);
        };
        const double pro = loss(+1.0, spin), retro = loss(-1.0, spin);
        const double still = loss(+1.0, 0.0), stillRetro = loss(-1.0, 0.0);
        // Without spin the two directions are mirror images
        CHECK(still == doctest::Approx(stillRetro).epsilon(1e-6));
        // With spin the prograde shuttle sees slower air; measured ratio 1.38
        CHECK(pro < still);
        CHECK(retro > still);
        CHECK(retro / pro > 1.25);
    }

    TEST_CASE("approaching the atmosphere at speed: the step is cut so that drag begins where the air begins")
    {
        // A fast shuttle (6 km/s) falling straight in from far above would otherwise cross the whole 98 km of air in one step.
        DomusWorld W(0.0);
        Shuttles s;
        REQUIRE(s.add(shuttleDesc(1, {kRDomus + 2.0e6, 0.0, 0.0}, {-6000.0, 0.0, 0.0})));
        StepRecorder rec;
        std::vector<Event> ev;
        propagateShuttle(s, 0, 0.0, 400.0, W.env, ev, &rec);
        const double top = W.env.atmosphereTop[0];
        // find the first step that ends inside the atmosphere reach
        std::size_t k = 0;
        while (k < rec.steps.size() && norm(rec.steps[k].r1) > kRDomus + top) ++k;
        REQUIRE(k < rec.steps.size());
        // The step that reached the layer was cut to the time of arrival plus one resolution step (0.1 H = 700 m of travel),
        // not left to run 100 km deep
        const double enteredDepth = (kRDomus + top) - norm(rec.steps[k].r1);
        CHECK(enteredDepth < 1500.0);
        CHECK(enteredDepth < 0.02 * top);
        CHECK(norm(shuttleVel(s, 0)) < 6000.0 + 5.0 * 9.8 * 400.0); // sanity: finite, not blown up
    }
}
