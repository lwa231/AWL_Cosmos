#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "core/impact.h"
#include "test_impact_util.h"

using namespace cosmos;
using namespace impact_test;

// Impact prediction against the parent from the osculating conic (plan 5.10). The reference for every hit time is a
// brute-force IAS15 integration of the two-body problem (epsilon 1e-11) plus bisection on its dense output.

namespace {
// Normalised units: mu = 1, trigger radius 1.
const double MU = 1.0;
const double RT = 1.0;

struct Case {
    const char* name;
    double a, e, omega, M;
    bool retro;
};

// Prediction vs. brute force for one orbit. Returns the relative error; the case must be an Impact.
double checkHit(const Case& c, double tol, double tMax)
{
    const RelState rs = stateFrom(c.a, c.e, c.omega, c.M, c.retro, MU);
    INFO(c.name << " r = " << norm(rs.r));
    REQUIRE(norm(rs.r) > RT);
    const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
    REQUIRE(pr.kind == ImpactKind::Impact);
    const auto ref = bruteParentCrossing(rs, MU, RT, tMax);
    REQUIRE(ref.has_value());
    const double err = relDiff(pr.t, *ref);
    INFO(c.name << ": predicted " << pr.t << " brute force " << *ref);
    CHECK(err < tol);
    return err;
}
} // namespace

TEST_SUITE("impact_parent")
{
    TEST_CASE("elliptic hits: inbound, outbound (wraps through periapsis), near apoapsis, rotated, retrograde")
    {
        // a = 3, e = 0.7: q = 0.9 < 1 < apoapsis 5.1; period 2 pi 3^1.5 = 32.6
        const Case cases[] = {
            {"inbound", 3.0, 0.7, 0.3, -2.0, false},
            {"inbound far", 3.0, 0.7, 2.0, -3.0, false},
            {"just before the crossing", 3.0, 0.7, 4.0, -0.35, false},
            {"outbound, wraps", 3.0, 0.7, 1.0, 1.0, false},
            {"outbound near apoapsis", 3.0, 0.7, -2.5, 3.0, false},
            {"at apoapsis", 3.0, 0.7, -2.5, 3.14159265358979, false},
            {"retro inbound", 3.0, 0.7, 0.3, -2.0, true},
            {"retro outbound", 3.0, 0.7, 5.0, 1.2, true},
            {"retro rotated", 3.0, 0.7, -1.7, -1.0, true},
            {"moderate e", 2.0, 0.55, 0.9, -1.5, false}, // q = 0.9
            {"grazing q", 4.0, 0.7501, 0.0, -2.5, false}, // q = 0.9996
            {"high e", 50.0, 0.9815, 2.2, -0.2, false},   // q = 0.925, apoapsis ~99
        };
        double worst = 0.0;
        for (const Case& c : cases) worst = std::max(worst, checkHit(c, 1e-11, 4000.0));
        std::printf("[impact] worst elliptic relative error: %.3e\n", worst);
    }

    TEST_CASE("hyperbolic hits: inbound branch, prograde and retrograde")
    {
        // a = -1, e = 1.5: q = 0.5 < 1 (r = 7.56 at M = -6, 1.89 at M = -1.2)
        const Case cases[] = {
            {"far inbound", -1.0, 1.5, 0.4, -6.0, false},
            {"near inbound", -1.0, 1.5, 0.4, -1.2, false},
            {"just outside", -1.0, 1.5, 3.0, -0.95, false}, // r = 1.54
            {"very far", -1.0, 1.5, 0.4, -60.0, false},
            {"retro far", -1.0, 1.5, 0.4, -6.0, true},
            {"retro near", -1.0, 1.5, -2.0, -1.2, true},
            {"strongly hyperbolic", -0.15, 6.0, 0.7, -6.0, false}, // q = 0.75, r = 1.23
            {"strongly hyperbolic far", -0.15, 6.0, 0.7, -60.0, true},
            {"nearly parabolic", -300.0, 1.003, 1.9, -0.05, false}, // q = 0.9
        };
        double worst = 0.0;
        for (const Case& c : cases) worst = std::max(worst, checkHit(c, 1e-11, 2000.0));
        std::printf("[impact] worst hyperbolic relative error: %.3e\n", worst);

        // periapsis above the trigger radius (q = 1.5): no hit
        const RelState rs = stateFrom(-0.5, 4.0, 1.0, -3.0, false, MU);
        CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
    }

    TEST_CASE("radial, parabolic and near-parabolic orbits (cancellation-prone limits)")
    {
        // Radial infall, bound (h = 0 exactly): r = 5, v = -0.3
        {
            const RelState rs{{5, 0, 0}, {-0.3, 0, 0}};
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            REQUIRE(pr.kind == ImpactKind::Impact);
            const auto ref = bruteParentCrossing(rs, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            CHECK(relDiff(pr.t, *ref) < 1e-11);
            CHECK(std::abs(pr.nuHit) == doctest::Approx(kPi)); // hits on the ray through the start point (nu = -pi)
            CHECK(pr.point.x == doctest::Approx(1.0).epsilon(1e-12));
            CHECK(std::abs(pr.point.y) < 1e-12);
        }
        // Radial, unbound
        {
            const RelState rs{{0, -5, 0}, {0, 1.2, 0}};
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            REQUIRE(pr.kind == ImpactKind::Impact);
            const auto ref = bruteParentCrossing(rs, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            CHECK(relDiff(pr.t, *ref) < 1e-11);
            CHECK(pr.point.y == doctest::Approx(-1.0).epsilon(1e-12));
        }
        // Radial parabola (zero energy): free fall from rest at infinity, t = (2/3)(r1^1.5 - r2^1.5) / sqrt(2 mu)
        {
            const double r0 = 5.0;
            const RelState rs{{r0, 0, 0}, {-std::sqrt(2.0 * MU / r0), 0, 0}};
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            REQUIRE(pr.kind == ImpactKind::Impact);
            const double exact = (2.0 / 3.0) * (std::pow(r0, 1.5) - std::pow(RT, 1.5)) / std::sqrt(2.0 * MU);
            CHECK(relDiff(pr.t, exact) < 1e-14);
            const auto ref = bruteParentCrossing(rs, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            CHECK(relDiff(pr.t, *ref) < 1e-11);
        }
        // Non-radial parabola (energy 0 to roundoff), and the same orbit nudged slightly bound / unbound: the three
        // are continuous (Barker vs the series forms)
        {
            const double r0 = 5.0, vt = 0.2; // q = (r0 vt)^2 / (2 mu) = 0.5
            const double vr = -std::sqrt(2.0 * MU / r0 - vt * vt);
            const RelState parab{{r0, 0, 0}, {vr, vt, 0}};
            const double tP = predictParentImpact(parab.r, parab.v, MU, RT).t;
            const auto ref = bruteParentCrossing(parab, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            CHECK(relDiff(tP, *ref) < 1e-11);
            for (double d : {-1e-9, -1e-12, -1e-14, 1e-14, 1e-12, 1e-9}) {
                const RelState rs{{r0, 0, 0}, {vr * (1.0 + d), vt * (1.0 + d), 0}};
                const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
                REQUIRE(pr.kind == ImpactKind::Impact);
                // dt/dv ~ O(1) here, so the time moves by about d; allow 10 d plus roundoff
                CHECK(std::abs(pr.t - tP) < 10.0 * std::abs(d) * tP + 1e-12 * tP);
            }
        }
        // Near-radial bound ellipse (p ~ 1e-6, tiny angular momentum): still matches the integration
        {
            const RelState rs{{5, 0, 0}, {-0.3, 1e-4, 0}};
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            REQUIRE(pr.kind == ImpactKind::Impact);
            const auto ref = bruteParentCrossing(rs, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            CHECK(relDiff(pr.t, *ref) < 1e-11);
        }
    }

    TEST_CASE("the crossing geometry: |point| = R_trig, nu_hit inbound, speed from vis-viva")
    {
        const Case c = {"geom", 3.0, 0.7, 1.1, -2.0, false};
        for (bool retro : {false, true}) {
            const RelState rs = stateFrom(c.a, c.e, c.omega, c.M, retro, MU);
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            REQUIRE(pr.kind == ImpactKind::Impact);
            CHECK(norm(pr.point) == doctest::Approx(RT).epsilon(1e-14));
            CHECK(pr.nuHit < 0.0);
            CHECK(pr.nuHit > -kPi);
            // cos nu_hit = (p / R - 1) / e
            const Elements el = stateToElements(rs.r, rs.v, MU);
            CHECK(std::cos(pr.nuHit) == doctest::Approx((el.p / RT - 1.0) / el.e).epsilon(1e-12));
            // Speed at the crossing: v^2 = mu (2/R - 1/a)
            CHECK(pr.speedHit == doctest::Approx(std::sqrt(MU * (2.0 / RT - 1.0 / c.a))).epsilon(1e-12));
            // Propagate the state by the predicted time with the exact Kepler solution: it must sit at the point
            const auto ref = bruteParentCrossing(rs, MU, RT, 100.0);
            REQUIRE(ref.has_value());
            Bodies b;
            b.add(point(MU, {0, 0, 0}, {0, 0, 0}));
            b.add(point(0.0, rs.r, rs.v));
            BodyState s = b.state;
            IAS15 ias;
            const ForceFn force = [&b](const BodyState& st, Accel& a) { computeAccelerations(st, b.mu, a); };
            double t = 0.0;
            while (t < pr.t) ias.step(s, force, t, pr.t);
            CHECK(std::abs(s.x[1] - pr.point.x) < 1e-8);
            CHECK(std::abs(s.y[1] - pr.point.y) < 1e-8);
        }
    }

    TEST_CASE("no impact: circular above R_trig, periapsis above R_trig, outbound hyperbola")
    {
        // circular at r = 2
        {
            const RelState rs = stateFrom(2.0, 0.0, 0.5, 1.0, false, MU);
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            CHECK(pr.kind == ImpactKind::None);
            CHECK(std::isinf(pr.t));
        }
        // circular just above, retrograde
        {
            const RelState rs = stateFrom(1.000001, 0.0, 0.5, 1.0, true, MU);
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
        }
        // eccentric, periapsis 1.2 > 1
        {
            const RelState rs = stateFrom(3.0, 0.6, 0.5, -1.0, false, MU); // q = 1.2
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
        }
        // periapsis just above (q = 1 + 1e-9) and just below (q = 1 - 1e-9)
        {
            const RelState above = stateFrom(3.0, 0.666666666, 0.5, -1.0, false, MU);
            const RelState below = stateFrom(3.0, 0.666666667, 0.5, -1.0, false, MU);
            CHECK(stateToElements(above.r, above.v, MU).q > RT);
            CHECK(stateToElements(below.r, below.v, MU).q < RT);
            CHECK(predictParentImpact(above.r, above.v, MU, RT).kind == ImpactKind::None);
            CHECK(predictParentImpact(below.r, below.v, MU, RT).kind == ImpactKind::Impact);
        }
        // hyperbola outbound (r.v > 0) with q < R_trig: already past periapsis, gone
        {
            const RelState rs = stateFrom(-1.0, 1.5, 0.4, 1.0, false, MU);
            REQUIRE(norm(rs.r) > RT);
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
            const RelState rr = stateFrom(-1.0, 1.5, 0.4, 1.0, true, MU);
            CHECK(predictParentImpact(rr.r, rr.v, MU, RT).kind == ImpactKind::None);
        }
        // hyperbola inbound but periapsis above R_trig
        {
            const RelState rs = stateFrom(-1.0, 2.5, 0.4, -3.0, false, MU); // q = 1.5
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
        }
        // outbound parabola
        {
            const double r0 = 3.0;
            const RelState rs{{r0, 0, 0}, {0.3, std::sqrt(2.0 * MU / r0 - 0.09), 0}};
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::None);
        }
    }

    TEST_CASE("inside cases: apoapsis below R_trig is InsideBound, otherwise Inside with t = 0")
    {
        const double rt = 5.0;
        // apoapsis a(1+e) = 2 * 1.5 = 3 <= 5, whole orbit inside (e.g. in the atmosphere): drag handles it
        {
            const RelState rs = stateFrom(2.0, 0.5, 0.3, 1.0, false, MU);
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, rt);
            CHECK(pr.kind == ImpactKind::InsideBound);
            CHECK(pr.t == 0.0);
            CHECK(!pr.hits());
            CHECK(!impactWithinLookAhead(pr, 1e6).has_value());
        }
        // apoapsis exactly R_trig: "<=" is inside-bound
        {
            const RelState rs = stateFrom(3.0, 2.0 / 3.0, 0.3, 0.2, false, MU);
            const Elements el = stateToElements(rs.r, rs.v, MU);
            CHECK(el.a * (1.0 + el.e) == doctest::Approx(5.0).epsilon(1e-12));
            const ImpactKind k = predictParentImpact(rs.r, rs.v, MU, 5.0 * (1.0 + 1e-9)).kind;
            CHECK(k == ImpactKind::InsideBound);
        }
        // currently inside but the orbit leaves R_trig later: Inside, t = 0
        {
            const RelState rs = stateFrom(4.0, 0.7, 0.3, 0.5, false, MU); // r near periapsis (1.2), apoapsis 6.8
            REQUIRE(norm(rs.r) < rt);
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, rt);
            CHECK(pr.kind == ImpactKind::Inside);
            CHECK(pr.t == 0.0);
            CHECK(impactWithinLookAhead(pr, 1e6, true).value() == 0.0);
            CHECK(!impactWithinLookAhead(pr, 1e6, false).has_value());
        }
        // unbound and inside
        {
            const RelState rs = stateFrom(-1.0, 1.5, 0.3, -0.5, false, MU);
            CHECK(predictParentImpact(rs.r, rs.v, MU, rt).kind == ImpactKind::Inside);
        }
        // exactly on the trigger sphere counts as inside
        {
            const RelState rs{{RT, 0, 0}, {-0.1, 0.9, 0}};
            CHECK(predictParentImpact(rs.r, rs.v, MU, RT).kind == ImpactKind::Inside);
        }
    }

    TEST_CASE("invalid input")
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(predictParentImpact({0, 0, 0}, {1, 0, 0}, MU, RT).kind == ImpactKind::Invalid);
        CHECK(predictParentImpact({2, 0, 0}, {nan, 0, 0}, MU, RT).kind == ImpactKind::Invalid);
        CHECK(predictParentImpact({2, 0, 0}, {0, 1, 0}, 0.0, RT).kind == ImpactKind::Invalid);
        CHECK(predictParentImpact({2, 0, 0}, {0, 1, 0}, -1.0, RT).kind == ImpactKind::Invalid);
        CHECK(predictParentImpact({2, 0, 0}, {0, 1, 0}, MU, 0.0).kind == ImpactKind::Invalid);
        CHECK(predictParentImpact({2, 0, 0}, {0, 1, 0}, MU, nan).kind == ImpactKind::Invalid);
    }

    TEST_CASE("mirror and 90 degree rotation of the whole orbit give bit-identical results")
    {
        // The timing uses only |r|, r.v, the energy and h^2, all invariant under these exact transformations.
        Mix rng{12345};
        for (int k = 0; k < 40; ++k) {
            const double a = rng.range(2.0, 6.0), e = rng.range(0.6, 0.85);
            const RelState rs = stateFrom(a, e, rng.range(-3, 3), rng.range(-3, 3), false, MU);
            if (norm(rs.r) <= RT) continue;
            const ImpactPrediction p0 = predictParentImpact(rs.r, rs.v, MU, RT);
            const ImpactPrediction pm = predictParentImpact({rs.r.x, -rs.r.y, 0}, {rs.v.x, -rs.v.y, 0}, MU, RT);
            const ImpactPrediction pr = predictParentImpact({-rs.r.y, rs.r.x, 0}, {-rs.v.y, rs.v.x, 0}, MU, RT);
            CHECK(p0.kind == pm.kind);
            CHECK(p0.kind == pr.kind);
            if (p0.kind != ImpactKind::Impact) continue;
            CHECK(p0.t == pm.t);
            CHECK(p0.t == pr.t);
            CHECK(p0.speedHit == pm.speedHit);
            // the crossing point mirrors / rotates with the orbit
            CHECK(std::abs(pm.point.x - p0.point.x) < 1e-12);
            CHECK(std::abs(pm.point.y + p0.point.y) < 1e-12);
            CHECK(std::abs(pr.point.x + p0.point.y) < 1e-12);
            CHECK(std::abs(pr.point.y - p0.point.x) < 1e-12);
        }
    }

    TEST_CASE("repeat calls are bit-identical (no hidden state)")
    {
        const RelState rs = stateFrom(3.0, 0.7, 0.3, -2.0, true, MU);
        const ImpactPrediction p1 = predictParentImpact(rs.r, rs.v, MU, RT);
        const ImpactPrediction p2 = predictParentImpact(rs.r, rs.v, MU, RT);
        CHECK(p1.t == p2.t);
        CHECK(p1.nuHit == p2.nuHit);
        CHECK(p1.point.x == p2.point.x);
    }

    TEST_CASE("random orbits against brute force (elliptic and hyperbolic, both senses)")
    {
        Mix rng{987654321};
        int hits = 0, checked = 0;
        double worst = 0.0;
        for (int k = 0; k < 60; ++k) {
            const bool hyper = (k % 3 == 2);
            const bool retro = (k % 2 == 1);
            double a, e, M;
            const double q = rng.range(0.2, 1.3); // periapsis: most orbits dip below R_trig = 1, a few do not
            if (hyper) {
                e = rng.range(1.05, 3.0);
                a = -q / (e - 1.0);
                M = -rng.range(0.2, 8.0); // inbound
            } else {
                e = rng.range(0.3, 0.97);
                a = q / (1.0 - e);
                M = rng.range(-kPi, kPi);
            }
            const RelState rs = stateFrom(a, e, rng.range(-kPi, kPi), M, retro, MU);
            const double r = norm(rs.r);
            if (r <= RT || !std::isfinite(r)) continue;
            const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, MU, RT);
            // Every orbit with q < RT that is not already inside is a hit if inbound or bound
            const bool shouldHit = q < RT && (!hyper || M < 0.0) && (hyper || a * (1.0 + e) > RT);
            ++checked;
            if (!shouldHit) {
                CHECK(pr.kind != ImpactKind::Impact);
                continue;
            }
            REQUIRE(pr.kind == ImpactKind::Impact);
            const double period = hyper ? 1e9 : 2.0 * kPi * std::pow(a, 1.5);
            const auto ref = bruteParentCrossing(rs, MU, RT, hyper ? 400.0 : 1.01 * period);
            REQUIRE(ref.has_value());
            const double err = relDiff(pr.t, *ref);
            worst = std::max(worst, err);
            CHECK_MESSAGE(err < 1e-11, "k=" << k << " a=" << a << " e=" << e << " M=" << M << " retro=" << retro);
            ++hits;
        }
        std::printf("[impact] random orbits: %d checked, %d hits, worst relative error %.3e\n", checked, hits, worst);
        CHECK(hits >= 20);
    }

    TEST_CASE("atmosphere: the trigger radius is surface plus top, in SI units (Earth-like)")
    {
        const double muE = 3.986004418e14, R = 6.371e6;
        Atmosphere at;
        at.rho0 = 1.225;
        at.scaleHeight = 7000.0;
        const std::optional<Atmosphere> atm = at;
        const double top = at.scaleHeight * std::log(at.rho0 / 1e-6);
        CHECK(triggerAtmosphereTop(atm) == doctest::Approx(top).epsilon(1e-14));
        CHECK(top == doctest::Approx(98.0e3).epsilon(0.01)); // ~98 km
        CHECK(triggerAtmosphereTop(std::nullopt) == 0.0);
        Atmosphere thin;
        thin.rho0 = 1e-7;
        thin.scaleHeight = 5000.0;
        CHECK(triggerAtmosphereTop(thin) == 0.0);
        CHECK(triggerRadius(R, atm) == doctest::Approx(R + top).epsilon(1e-14));
        CHECK(triggerRadius(R, std::nullopt) == R);

        // Elliptical orbit from 800 km altitude apoapsis, periapsis below the surface
        const double rp = R - 50e3, ra = R + 800e3; // periapsis 50 km under the surface: it enters the air, then hits
        const double a = 0.5 * (rp + ra), e = (ra - rp) / (ra + rp);
        const RelState rs = stateFrom(a, e, 0.8, -1.0, false, muE);
        const double rTrig = triggerRadius(R, atm);
        const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, muE, rTrig);
        REQUIRE(pr.kind == ImpactKind::Impact);
        const auto ref = bruteParentCrossing(rs, muE, rTrig, 2.0 * 2.0 * kPi * std::sqrt(a * a * a / muE));
        REQUIRE(ref.has_value());
        CHECK(relDiff(pr.t, *ref) < 1e-11);
        // The surface (R) is crossed later than the air (R + top)
        const ImpactPrediction surf = predictParentImpact(rs.r, rs.v, muE, R);
        REQUIRE(surf.kind == ImpactKind::Impact);
        CHECK(surf.t > pr.t);
        // Same via the shuttle convenience function (parent not at the origin and moving)
        const Vec3 pp{1.0e9, -2.0e9, 0.0}, vp{300.0, 100.0, 0.0};
        const ImpactPrediction viaShuttle = predictShuttleImpact(pp + rs.r, vp + rs.v, pp, vp, muE, R, atm);
        REQUIRE(viaShuttle.kind == ImpactKind::Impact);
        CHECK(relDiff(viaShuttle.t, pr.t) < 1e-6); // absolute positions at 1e9 m cost a few digits of the relative state
    }
}
