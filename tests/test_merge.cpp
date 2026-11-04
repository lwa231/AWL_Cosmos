#include <doctest.h>
#include <cmath>
#include <cstdio>
#include "core/bodies.h"
#include "core/constants.h"
#include "core/diagnostics.h"
#include "core/events.h"
#include "core/kinds.h"
#include "core/merge.h"

using namespace cosmos;

namespace {
BodyDesc body(const char* name, double mu, double radius, Vec3 p, Vec3 v, double spin = 0.0)
{
    BodyDesc d;
    d.name = name;
    d.mu = mu;
    d.radius = radius;
    d.position = p;
    d.velocity = v;
    d.spinRate = spin;
    return d;
}

// Sum of the magnitudes of every term that goes into P and L: the scale "relative" errors are measured against.
struct Scale { double p = 0.0, l = 0.0; };
Scale scaleOf(const Bodies& b)
{
    Scale s;
    for (std::size_t i = 0; i < b.size(); ++i) {
        const Vec3 r{b.state.x[i], b.state.y[i], b.state.z[i]};
        const Vec3 v{b.state.vx[i], b.state.vy[i], b.state.vz[i]};
        s.p += b.mu[i] * norm(v);
        s.l += b.mu[i] * norm(cross(r, v)) +
               std::abs(b.inertiaFactor[i] * b.mu[i] * b.radius[i] * b.radius[i] * b.spinRate[i]);
    }
    return s;
}

struct Errors { double p, l; };

// Merge a and b (IDs), then compare P and L (with the logged loss added) against before.
Errors mergeAndCompare(Bodies& b, BodyId ida, BodyId idb, MergeResult* out = nullptr)
{
    const Conserved before = conserved(b);
    const Scale sc = scaleOf(b);
    const auto m = mergeBodies(b, ida, idb, 1.0);
    REQUIRE(m.has_value());
    if (out) *out = *m;
    const Conserved after = conserved(b);
    const Vec3 dP = after.momentumG - before.momentumG;
    const Vec3 dL = (after.angularMomentumG + m->lostAngularMomentumG) - before.angularMomentumG;
    return {norm(dP) / sc.p, norm(dL) / sc.l};
}

// Two overlapping planar bodies in dimensionless units (mu ~ 1). omega_break of the result is about 31 rad/s.
Bodies pair(double speed)
{
    Bodies b;
    b.add(body("A", 1.0, 0.10, {0.0, 0.0, 0.0}, {0.1, -speed, 0.0}, 0.5));
    b.add(body("B", 0.5, 0.08, {0.15, 0.05, 0.0}, {-0.3, speed, 0.0}, -0.2));
    return b;
}
}

TEST_SUITE("merge")
{
    TEST_CASE("below omega_break: P and L (incl. spin) conserved to 1e-14, nothing clamped")
    {
        Bodies b = pair(0.3);
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        std::printf("[validation] merge below omega_break: |omega_raw| %.4g < omega_break %.4g; P error %.3g, L error %.3g, dL logged %.3g, dE logged %.6g\n",
                    std::abs(m.omegaRaw), m.omegaBreak, e.p, e.l, m.lostAngularMomentumG.z, m.dEnergyG);
        CHECK(!m.spinClamped);
        CHECK(std::abs(m.omegaRaw) < m.omegaBreak);
        CHECK(m.lostAngularMomentumG.z == 0.0);
        CHECK(m.lostAngularMomentumG.x == 0.0);
        CHECK(m.lostAngularMomentumG.y == 0.0);
        CHECK(e.p <= 1e-14);
        CHECK(e.l <= 1e-14);
        // the stored spin is exactly the unclamped one
        CHECK(b.spinRate[0] == m.omegaRaw);
    }

    TEST_CASE("above omega_break: spin clamped, loss logged, P and L + logged loss conserved to 1e-14")
    {
        Bodies b = pair(30.0);
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        std::printf("[validation] merge above omega_break: |omega_raw| %.4g > omega_break %.4g; P error %.3g, L error (logged dL added back) %.3g, dL logged %.6g, dE logged %.6g\n",
                    std::abs(m.omegaRaw), m.omegaBreak, e.p, e.l, m.lostAngularMomentumG.z, m.dEnergyG);
        REQUIRE(m.spinClamped);
        CHECK(std::abs(m.omegaRaw) > m.omegaBreak);
        CHECK(std::abs(b.spinRate[0]) == m.omegaBreak);
        CHECK(std::signbit(b.spinRate[0]) == std::signbit(m.omegaRaw));
        CHECK(m.lostAngularMomentumG.z != 0.0);
        // lost L has the sign of the raw spin and equals I (omega_raw - omega_break) in magnitude
        const double I = 0.4 * b.mu[0] * b.radius[0] * b.radius[0];
        CHECK(m.lostAngularMomentumG.z == doctest::Approx(I * (m.omegaRaw - std::copysign(m.omegaBreak, m.omegaRaw))).epsilon(1e-13).scale(0));
        CHECK(e.p <= 1e-14);
        CHECK(e.l <= 1e-14);
        // without adding the loss back, L is visibly off
        const Conserved after = conserved(b);
        CHECK(std::abs(m.lostAngularMomentumG.z) > 1e-3 * std::abs(after.angularMomentumG.z));
    }

    TEST_CASE("clamp in the negative direction")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.10, {0.0, 0.0, 0.0}, {0.0, 30.0, 0.0}));
        b.add(body("B", 0.5, 0.08, {0.15, 0.05, 0.0}, {0.0, -30.0, 0.0}));
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        REQUIRE(m.spinClamped);
        CHECK(m.omegaRaw < 0.0);
        CHECK(b.spinRate[0] == -m.omegaBreak);
        CHECK(e.l <= 1e-14);
    }

    TEST_CASE("momentum and angular momentum are conserved over many random-ish planar and 3D merges")
    {
        // deterministic pseudo-random scenarios from a fixed formula (no RNG)
        double worstP = 0.0, worstL = 0.0;
        for (int k = 0; k < 200; ++k) {
            const double f = 0.37 * k + 0.11;
            Bodies b;
            b.add(body("A", 1.0 + std::sin(f), 0.2 + 0.1 * std::cos(f), {0.1 * std::sin(2 * f), 0.1 * std::cos(3 * f), 0.05 * std::sin(f)},
                       {std::cos(f), std::sin(1.7 * f), 0.3 * std::cos(2.3 * f)}, 2.0 * std::sin(5 * f)));
            b.add(body("B", 0.5 + 0.4 * std::cos(f), 0.15 + 0.05 * std::sin(f), {0.2 * std::cos(f), 0.1 * std::sin(f), -0.04 * std::cos(2 * f)},
                       {std::sin(f), -std::cos(1.3 * f), -0.2 * std::sin(f)}, 3.0 * std::cos(4 * f)));
            // k even: planar (z = 0 exactly); k odd: fully 3D
            if (k % 2 == 0) {
                b.state.z = {0.0, 0.0};
                b.state.vz = {0.0, 0.0};
            }
            const Errors e = mergeAndCompare(b, 1, 2);
            worstP = std::max(worstP, e.p);
            worstL = std::max(worstL, e.l);
        }
        // measured worst case 4.8e-16 over these 200 merges (a few ulps); the bound is the required 1e-14
        std::printf("[validation] merge, 200 random planar/3D merges: worst P error %.3g, worst L error %.3g\n", worstP, worstL);
        CHECK(worstP <= 1e-14);
        CHECK(worstL <= 1e-14);
    }

    TEST_CASE("3D: orbital L out of the plane cannot be stored as a z spin and is logged")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.1, {0, 0, 0}, {0, 0, 1.0}));
        b.add(body("B", 1.0, 0.1, {0.1, 0, 0}, {0, 0, -1.0}));
        // relative orbital L about x/y: nonzero, spin is about z only
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        CHECK(m.lostAngularMomentumG.y != 0.0);
        CHECK(e.l <= 1e-14);
    }

    TEST_CASE("point masses (radius 0): no spin can be held, all orbital L is logged")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.0, {0, 0, 0}, {0, 0.5, 0}));
        b.add(body("B", 1.0, 0.0, {0, 0, 0}, {0, -0.5, 0})); // coincident, so overlapping
        b.state.x[1] = 1e-3;
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        CHECK(b.spinRate[0] == 0.0);
        CHECK(b.radius[0] == 0.0);
        CHECK(m.lostAngularMomentumG.z != 0.0);
        CHECK(e.l <= 1e-14);
        CHECK(std::isfinite(b.state.x[0]));
    }

    TEST_CASE("mass, position, velocity are mu-weighted; radius adds by volume; I_G = k mu R^2")
    {
        Bodies b = pair(0.3);
        const BodyId s = 1;
        const auto m = mergeBodies(b, 1, 2, 4.0);
        REQUIRE(m.has_value());
        REQUIRE(b.size() == 1);
        const std::size_t i = *b.find(s);
        CHECK(b.mu[i] == 1.5);
        CHECK(b.state.x[i] == doctest::Approx((1.0 * 0.0 + 0.5 * 0.15) / 1.5).epsilon(1e-15).scale(0));
        CHECK(b.state.y[i] == doctest::Approx((0.5 * 0.05) / 1.5).epsilon(1e-15).scale(0));
        CHECK(b.state.vx[i] == doctest::Approx((1.0 * 0.1 + 0.5 * -0.3) / 1.5).epsilon(1e-15).scale(0));
        CHECK(b.state.vy[i] == doctest::Approx((1.0 * -0.3 + 0.5 * 0.3) / 1.5).epsilon(1e-14).scale(1e-3));
        CHECK(b.radius[i] == doctest::Approx(std::cbrt(0.1 * 0.1 * 0.1 + 0.08 * 0.08 * 0.08)).epsilon(1e-15).scale(0));
        CHECK(b.inertiaFactor[i] == 0.4);
        // omega = S_G / I_G: reproduce L from the stored spin
        CHECK(b.spinRate[i] == m->omegaRaw);
        CHECK(m->omegaBreak == doctest::Approx(std::sqrt(b.mu[i] / (b.radius[i] * b.radius[i] * b.radius[i]))).epsilon(1e-15).scale(0));
    }

    TEST_CASE("survivor keeps the heavier body's ID, name, colour, atmosphere, flags; absorbed is gone")
    {
        Bodies b;
        BodyDesc heavy = body("Heavy", 3.0, 1.0, {0, 0, 0}, {0, 0, 0});
        heavy.kind = BodyKind::GasGiant;
        heavy.kindLocked = true;
        heavy.fictionalDensity = true;
        heavy.atmosphere = Atmosphere{1.5, 8000.0};
        heavy.colour = {0.1f, 0.2f, 0.3f};
        heavy.inertiaFactor = 0.25;
        BodyDesc light = body("Light", 1.0, 0.5, {1.0, 0, 0}, {0, 0, 0});
        light.kind = BodyKind::Moon;
        light.atmosphere = Atmosphere{9.0, 1.0};
        const BodyId lightId = b.add(light); // added first so its ID is lower
        const BodyId heavyId = b.add(heavy);

        const auto m = mergeBodies(b, lightId, heavyId, 2.0);
        REQUIRE(m.has_value());
        CHECK(m->survivor == heavyId);
        CHECK(m->absorbed == lightId);
        CHECK(b.size() == 1);
        CHECK(b.consistent());
        CHECK(!b.find(lightId).has_value());
        const std::size_t i = *b.find(heavyId);
        CHECK(b.name[i] == "Heavy");
        CHECK(b.kind[i] == BodyKind::GasGiant);
        CHECK(b.kindLocked[i] == 1);
        CHECK(b.fictionalDensity[i] == 1);
        REQUIRE(b.atmosphere[i].has_value());
        CHECK(b.atmosphere[i]->rho0 == 1.5);
        CHECK(b.atmosphere[i]->scaleHeight == 8000.0);
        CHECK(b.colour[i][2] == 0.3f);
        CHECK(b.inertiaFactor[i] == 0.25);

        // IDs are never reused: the next body gets a new one
        const BodyId next = b.add(body("New", 1.0, 0, {}, {}));
        CHECK(next != lightId);
        CHECK(next != heavyId);
    }

    TEST_CASE("argument order does not matter; equal mu keeps the lower ID")
    {
        Bodies b1 = pair(0.3), b2 = pair(0.3);
        mergeBodies(b1, 1, 2, 1.0);
        mergeBodies(b2, 2, 1, 1.0);
        CHECK(b1.ids == b2.ids);
        CHECK(b1.mu[0] == b2.mu[0]);
        CHECK(b1.state.x[0] == b2.state.x[0]);
        CHECK(b1.spinRate[0] == b2.spinRate[0]);

        Bodies t;
        t.add(body("first", 1.0, 0.1, {0, 0, 0}, {}));
        t.add(body("second", 1.0, 0.1, {0.1, 0, 0}, {}));
        const auto m = mergeBodies(t, 2, 1, 0.0);
        REQUIRE(m.has_value());
        CHECK(m->survivor == 1);
        CHECK(t.name[0] == "first");
    }

    TEST_CASE("invalid merges change nothing")
    {
        Bodies b = pair(0.3);
        CHECK(!mergeBodies(b, 1, 1, 0.0).has_value());
        CHECK(!mergeBodies(b, 1, 99, 0.0).has_value());
        CHECK(!mergeBodies(b, 99, 2, 0.0).has_value());
        CHECK(b.size() == 2);
    }

    TEST_CASE("dE is logged, matches the two-body formula, and adding it back cancels")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.10, {0.0, 0.0, 0.0}, {0.1, -0.3, 0.0}));
        b.add(body("B", 0.5, 0.08, {0.06, 0.02, 0.0}, {-0.3, 0.3, 0.0}));
        const Conserved before = conserved(b);
        const double rel = norm(Vec3{-0.4, 0.6, 0.0});
        const double r = norm(Vec3{0.06, 0.02, 0.0});
        MergeResult m;
        REQUIRE(r < 0.10); // inside R_c: the section 5.3 potential applies to the pair
        {
            const auto res = mergeBodies(b, 1, 2, 1.0);
            REQUIRE(res.has_value());
            m = *res;
        }
        const Conserved after = conserved(b);

        CHECK(m.dEnergyG == after.energyG - before.energyG);
        // exact for two bodies: lost kinetic energy (reduced mass) and removed pair potential
        const double keLost = 0.5 * (1.0 * 0.5 / 1.5) * rel * rel;
        const double uBefore = pairPotentialG(1.0, 0.5, r, 0.10);
        CHECK(m.dEnergyG == doctest::Approx(-keLost - uBefore).epsilon(1e-13).scale(0));
        CHECK(uBefore > -1.0 * 0.5 / r); // shallower than the point-mass value, as section 5.3 says

        MergeLedger led;
        record(led, m);
        const Conserved fixedUp = withMergesAddedBack(after, led);
        CHECK(fixedUp.energyG == doctest::Approx(before.energyG).epsilon(1e-14).scale(0));
        CHECK(fixedUp.momentumG.x == doctest::Approx(before.momentumG.x).epsilon(1e-14).scale(1e-3));
        CHECK(fixedUp.angularMomentumG.z == doctest::Approx(before.angularMomentumG.z).epsilon(1e-14).scale(1e-3));
    }

    TEST_CASE("dE uses the full system: a third body's potential changes with the merge")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.10, {0.0, 0.0, 0.0}, {0.1, -0.3, 0.0}));
        b.add(body("B", 0.5, 0.08, {0.06, 0.02, 0.0}, {-0.3, 0.3, 0.0}));
        b.add(body("C", 2.0, 0.05, {1.0, 0.3, 0.0}, {0.0, 0.2, 0.0}));
        const Conserved before = conserved(b);
        const auto m = mergeBodies(b, 1, 2, 1.0);
        REQUIRE(m.has_value());
        const Conserved after = conserved(b);
        CHECK(m->dEnergyG == after.energyG - before.energyG);
        CHECK(after.momentumG.x == doctest::Approx(before.momentumG.x).epsilon(1e-14).scale(1e-2));

        // differs from the isolated pair's dE because C's potential energy changes
        Bodies pairOnly;
        pairOnly.add(body("A", 1.0, 0.10, {0.0, 0.0, 0.0}, {0.1, -0.3, 0.0}));
        pairOnly.add(body("B", 0.5, 0.08, {0.06, 0.02, 0.0}, {-0.3, 0.3, 0.0}));
        const auto p = mergeBodies(pairOnly, 1, 2, 1.0);
        REQUIRE(p.has_value());
        CHECK(m->dEnergyG != p->dEnergyG);

        MergeLedger led;
        record(led, *m);
        CHECK(withMergesAddedBack(after, led).energyG == doctest::Approx(before.energyG).epsilon(1e-14).scale(0));
    }

    TEST_CASE("a chain of merges accumulates in the ledger and still cancels")
    {
        Bodies b;
        b.add(body("A", 1.0, 0.10, {0.00, 0.0, 0.0}, {0.1, -0.3, 0.0}, 0.3));
        b.add(body("B", 0.5, 0.08, {0.15, 0.05, 0.0}, {-0.3, 0.3, 0.0}, -0.1));
        b.add(body("C", 0.7, 0.09, {0.10, 0.20, 0.0}, {0.2, 0.9, 0.0}, 0.2));
        b.add(body("D", 3.0, 0.20, {5.0, 5.0, 0.0}, {0.0, -0.1, 0.0}));
        const Conserved before = conserved(b);
        MergeLedger led;
        for (BodyId other : {BodyId(2), BodyId(3)}) {
            const auto m = mergeBodies(b, 1, other, 1.0);
            REQUIRE(m.has_value());
            record(led, *m);
        }
        REQUIRE(b.size() == 2);
        const Conserved fixedUp = withMergesAddedBack(conserved(b), led);
        const Scale sc = scaleOf(b);
        CHECK(fixedUp.energyG == doctest::Approx(before.energyG).epsilon(1e-13).scale(0));
        CHECK(std::abs(fixedUp.angularMomentumG.z - before.angularMomentumG.z) <= 1e-14 * (sc.l + 8.0));
        CHECK(norm(fixedUp.momentumG - before.momentumG) <= 1e-14 * (sc.p + 1.0));
    }

    TEST_CASE("events: Collision, Merge (and KindChanged) are emitted with the right payloads")
    {
        Bodies b;
        b.add(body("A", 3.0, 3.0, {0, 0, 0}, {1.0, 0, 0}));
        b.add(body("B", 1.0, 1.0, {4.0, 0, 0}, {-1.0, 0, 0}));
        EventQueue q;
        const auto m = mergeBodies(b, 2, 1, 7.25, &q);
        REQUIRE(m.has_value());
        q.sort();
        REQUIRE(q.size() == 2); // no reclassification: Planet + Planet
        const Event& c = q.events()[0];
        const Event& g = q.events()[1];
        CHECK(c.type == EventType::Collision);
        CHECK(c.t == 7.25);
        CHECK(c.id == 1);
        CHECK(c.other == 2);
        CHECK(c.relSpeed == 2.0);
        CHECK(c.point.x == 3.0); // touching pair: on the line of centres, 3 m (= R_A) from A
        CHECK(c.point.y == 0.0);
        CHECK(g.type == EventType::Merge);
        CHECK(g.t == 7.25);
        CHECK(g.id == 1);
        CHECK(g.other == 2);
    }

    TEST_CASE("neutron stars merging above 2.3 solar masses become a black hole: R = 2 mu / c^2, KindChanged emitted")
    {
        Bodies b;
        BodyDesc a = defaultBodyDesc(BodyKind::NeutronStar);
        a.name = "NS A";
        a.position = {0, 0, 0};
        BodyDesc c = defaultBodyDesc(BodyKind::NeutronStar);
        c.name = "NS B";
        c.mu = 1.0 * kSolarMu;
        c.position = {1.0e4, 0, 0};
        c.velocity = {-1.0e5, 0, 0};
        const BodyId ia = b.add(a), ic = b.add(c);

        EventQueue q;
        const auto m = mergeBodies(b, ia, ic, 3.0, &q);
        REQUIRE(m.has_value());
        const std::size_t i = *b.find(ia);
        CHECK(b.mu[i] == 2.4 * kSolarMu);
        CHECK(b.kind[i] == BodyKind::BlackHole);
        CHECK(m->oldKind == BodyKind::NeutronStar);
        CHECK(m->newKind == BodyKind::BlackHole);
        CHECK(b.radius[i] == 2.0 * b.mu[i] / (constants::c * constants::c));
        CHECK(b.radius[i] < 1.2e4); // shrank far below the neutron-star radius

        q.sort();
        REQUIRE(q.size() == 3);
        CHECK(q.events()[0].type == EventType::Collision);
        CHECK(q.events()[1].type == EventType::Merge);
        CHECK(q.events()[2].type == EventType::KindChanged);
        CHECK(q.events()[2].id == ia);
        CHECK(q.events()[2].oldKind == BodyKind::NeutronStar);
        CHECK(q.events()[2].newKind == BodyKind::BlackHole);
    }

    TEST_CASE("a neutron star pair at or below 2.3 solar masses stays a neutron star with volume-added radius")
    {
        Bodies b;
        BodyDesc a = defaultBodyDesc(BodyKind::NeutronStar);
        a.mu = 1.1 * kSolarMu;
        BodyDesc c = a;
        c.position = {1.0e4, 0, 0};
        const BodyId ia = b.add(a), ic = b.add(c);
        EventQueue q;
        REQUIRE(mergeBodies(b, ia, ic, 0.0, &q).has_value());
        CHECK(b.kind[0] == BodyKind::NeutronStar);
        CHECK(b.radius[0] == doctest::Approx(1.2e4 * std::cbrt(2.0)).epsilon(1e-14).scale(0));
        CHECK(q.size() == 2);
    }

    TEST_CASE("anything merging with a black hole becomes (stays) a black hole")
    {
        Bodies b;
        BodyDesc bh = defaultBodyDesc(BodyKind::BlackHole);
        bh.position = {0, 0, 0};
        BodyDesc star = defaultBodyDesc(BodyKind::Star); // heavier than... no: 1 Msun < 10 Msun, BH survives
        star.position = {1.0e8, 0, 0};
        const BodyId ibh = b.add(bh), is = b.add(star);
        EventQueue q;
        const auto m = mergeBodies(b, is, ibh, 0.0, &q);
        REQUIRE(m.has_value());
        CHECK(m->survivor == ibh);
        CHECK(b.kind[0] == BodyKind::BlackHole);
        CHECK(b.radius[0] == 2.0 * b.mu[0] / (constants::c * constants::c));
        CHECK(q.size() == 2); // BH -> BH: no KindChanged

        // a heavier non-locked star swallowing a black hole is reclassified
        Bodies b2;
        BodyDesc smallBH = defaultBodyDesc(BodyKind::BlackHole);
        smallBH.mu = 0.5 * kSolarMu;
        BodyDesc big = defaultBodyDesc(BodyKind::Star);
        big.position = {1.0e8, 0, 0};
        const BodyId sb = b2.add(smallBH), bg = b2.add(big);
        const auto m2 = mergeBodies(b2, sb, bg, 0.0);
        REQUIRE(m2.has_value());
        CHECK(m2->survivor == bg);
        CHECK(m2->oldKind == BodyKind::Star);
        CHECK(m2->newKind == BodyKind::BlackHole);
        CHECK(b2.kind[0] == BodyKind::BlackHole);
    }

    TEST_CASE("kindLocked survivors keep their kind (Lucerna's sub-stellar star)")
    {
        Bodies b;
        BodyDesc star = defaultBodyDesc(BodyKind::Star);
        star.name = "Lucerna";
        star.kindLocked = true;
        star.mu = 1.25e18;
        star.radius = 1.0e6;
        BodyDesc bh = defaultBodyDesc(BodyKind::BlackHole);
        bh.mu = 1.0e15; // lighter: the star survives
        bh.radius = 1.0;
        bh.position = {5.0e5, 0, 0};
        const BodyId s = b.add(star), h = b.add(bh);
        EventQueue q;
        const auto m = mergeBodies(b, s, h, 0.0, &q);
        REQUIRE(m.has_value());
        CHECK(m->survivor == s);
        CHECK(b.kind[0] == BodyKind::Star);
        CHECK(q.size() == 2);
    }

    TEST_CASE("black-hole merge conserves P and L with the logged spin handling")
    {
        Bodies b;
        BodyDesc a = defaultBodyDesc(BodyKind::BlackHole);
        a.spinRate = 1.0e-3;
        BodyDesc c = defaultBodyDesc(BodyKind::BlackHole);
        c.mu = 6.0 * kSolarMu;
        c.position = {4.0e4, 1.0e4, 0};
        c.velocity = {-2.0e7, 1.0e6, 0};
        b.add(a);
        b.add(c);
        MergeResult m;
        const Errors e = mergeAndCompare(b, 1, 2, &m);
        CHECK(e.p <= 1e-14);
        CHECK(e.l <= 1e-14);
        CHECK(b.radius[0] == 2.0 * b.mu[0] / (constants::c * constants::c));
    }
}
