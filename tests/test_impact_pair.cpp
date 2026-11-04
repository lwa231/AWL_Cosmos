#include <doctest.h>
#include <cmath>
#include <limits>
#include "core/impact.h"
#include "test_impact_util.h"

using namespace cosmos;
using namespace impact_test;

// Body-body pair prediction, the rough closing estimate and the look-ahead (plan 5.10, 8 item 4).

TEST_SUITE("impact_pair")
{
    TEST_CASE("pair prediction matches an N-body integration of the two bodies (elliptic, hyperbolic, retrograde)")
    {
        struct PairCase {
            const char* name;
            double a, e, omega, M;
            bool retro;
        };
        // mu_a = 3, mu_b = 1 -> relative orbit mu = 4; contact distance R_a + R_b = 1.0
        const PairCase cases[] = {
            {"ellipse", 4.0, 0.8, 0.5, -1.8, false}, // q = 0.8
            {"ellipse outbound", 4.0, 0.8, -2.0, 1.0, false},
            {"ellipse retro", 4.0, 0.8, 2.5, -1.8, true},
            {"hyperbola", -1.2, 1.5, 0.9, -3.0, false}, // q = 0.6
            {"hyperbola retro", -1.2, 1.5, -2.9, -3.0, true},
        };
        for (const PairCase& c : cases) {
            INFO(c.name);
            const double muA = 3.0, muB = 1.0, Ra = 0.4, Rb = 0.6;
            const RelState rs = stateFrom(c.a, c.e, c.omega, c.M, c.retro, muA + muB);
            // Both bodies far from the origin and drifting: only the relative orbit matters
            const Vec3 pa{100.0, -40.0, 0.0}, va{0.3, 0.1, 0.0};
            Bodies b;
            b.add(point(muA, pa, va, Ra));
            b.add(point(muB, pa + rs.r, va + rs.v, Rb));

            const ImpactPrediction pr = predictPairImpact(b, 0, 1);
            const ImpactPrediction pr2 = predictPairImpact(pa, va, muA, Ra, pa + rs.r, va + rs.v, muB, Rb);
            REQUIRE(pr.kind == ImpactKind::Impact);
            CHECK(pr.t == pr2.t);
            const auto ref = bruteCrossing(b, 0, 1, Ra + Rb, 2000.0);
            REQUIRE(ref.has_value());
            // The positions are rounded at |x| ~ 100, which costs ~1e-14 of the relative state (measured error ~1e-14)
            CHECK(relDiff(pr.t, *ref) < 1e-10);
            // the contact point is relative to body a
            CHECK(norm(pr.point) == doctest::Approx(Ra + Rb).epsilon(1e-12));
        }
    }

    TEST_CASE("pair that misses gives None; swapping a and b gives the same time")
    {
        const double muA = 3.0, muB = 1.0;
        const RelState rs = stateFrom(4.0, 0.8, 0.5, -1.8, false, muA + muB);
        const ImpactPrediction ab = predictPairImpact({0, 0, 0}, {0, 0, 0}, muA, 0.4, rs.r, rs.v, muB, 0.6);
        const ImpactPrediction ba = predictPairImpact(rs.r, rs.v, muB, 0.6, {0, 0, 0}, {0, 0, 0}, muA, 0.4);
        REQUIRE(ab.kind == ImpactKind::Impact);
        REQUIRE(ba.kind == ImpactKind::Impact);
        // r -> -r, v -> -v: the same orbit seen from the other body; exact in |r|, r.v, energy and h^2
        CHECK(ab.t == ba.t);
        // outer pair on a circular orbit
        const RelState circ = stateFrom(5.0, 0.0, 0.2, 0.4, false, muA + muB);
        CHECK(predictPairImpact({0, 0, 0}, {0, 0, 0}, muA, 0.4, circ.r, circ.v, muB, 0.6).kind == ImpactKind::None);
    }

    TEST_CASE("predictPairImpacts: filters by horizon, sorted by (t, ids), reusable output")
    {
        // Heavy centre (mu = 1, R = 1) at rest; massless bodies dropped from rest at r = 20, 10, 30; a far bystander.
        Bodies b;
        b.add(point(1.0, {0, 0, 0}, {0, 0, 0}, 1.0));
        b.add(point(0.0, {20, 0, 0}, {0, 0, 0}, 0.0));
        b.add(point(0.0, {0, 10, 0}, {0, 0, 0}, 0.0));
        b.add(point(0.0, {-30, 0, 0}, {0, 0, 0}, 0.0));
        b.add(point(1e-3, {1000, 1000, 0}, {0, 0, 0}, 0.0));
        std::vector<PairImpact> out;
        out.reserve(8);
        const PairImpact* before = out.data();
        predictPairImpacts(b, 1e9, out);
        REQUIRE(out.size() >= 3);
        // Free fall from rest at r0 to R (mu = 1): t = sqrt(r0^3 / 2) (acos(sqrt(x)) + sqrt(x (1 - x))), x = R / r0
        auto fall = [](double r0, double R) {
            const double x = R / r0;
            return std::sqrt(r0 * r0 * r0 / 2.0) * (std::acos(std::sqrt(x)) + std::sqrt(x * (1.0 - x)));
        };
        // pairs (0,2) from r = 10, (0,1) from r = 20, (0,3) from r = 30, in that order of time
        REQUIRE(out[0].ib == 2);
        REQUIRE(out[1].ib == 1);
        REQUIRE(out[2].ib == 3);
        CHECK(out[0].prediction.t == doctest::Approx(fall(10.0, 1.0)).epsilon(1e-9));
        CHECK(out[1].prediction.t == doctest::Approx(fall(20.0, 1.0)).epsilon(1e-9));
        CHECK(out[2].prediction.t == doctest::Approx(fall(30.0, 1.0)).epsilon(1e-9));
        for (std::size_t i = 1; i < out.size(); ++i) CHECK(out[i - 1].prediction.t <= out[i].prediction.t);
        // A horizon between the first two arrivals keeps only the first
        predictPairImpacts(b, 0.5 * (out[0].prediction.t + out[1].prediction.t), out);
        REQUIRE(out.size() == 1);
        CHECK(out[0].ib == 2);
        predictPairImpacts(b, 1.0, out);
        CHECK(out.empty());
        CHECK(out.data() == before); // the reserved capacity was kept: no reallocation
    }

    TEST_CASE("closing estimate: exact for constant-acceleration motion along the line of centres")
    {
        // Gap 100, closing speed 5, closing acceleration 2: t^2 + 5 t - 100 = 0
        const double t1 = 0.5 * (-5.0 + std::sqrt(25.0 + 400.0));
        const auto c = closingTime(100.0, 5.0, 2.0);
        REQUIRE(c.has_value());
        CHECK(*c == doctest::Approx(t1).epsilon(1e-14));
        CHECK(5.0 * *c + 0.5 * 2.0 * *c * *c == doctest::Approx(100.0).epsilon(1e-13));

        // Two bodies on a line with constant accelerations
        const double xa0 = -30.0, va = 3.0, aa = 0.5;
        const double xb0 = 70.0, vb = -1.0, ab = -0.25, R = 4.0; // gap 96, vc = 4, ac = 0.75
        const auto est = estimateClosing({xb0 - xa0, 0, 0}, {vb - va, 0, 0}, {ab - aa, 0, 0}, R);
        REQUIRE(est.has_value());
        const double t = *est;
        const double xa = xa0 + va * t + 0.5 * aa * t * t;
        const double xb = xb0 + vb * t + 0.5 * ab * t * t;
        CHECK(xb - xa == doctest::Approx(R).epsilon(1e-13));
        // it is the first time: the gap is still positive just before
        const double te = 0.999 * t;
        CHECK((xb0 + vb * te + 0.5 * ab * te * te) - (xa0 + va * te + 0.5 * aa * te * te) > R);

        // The same along a rotated line (any direction): the vector form gives the same time
        const double ang = 0.7, ca = std::cos(ang), sa = std::sin(ang);
        const auto rot = estimateClosing({ca * (xb0 - xa0), sa * (xb0 - xa0), 0}, {ca * (vb - va), sa * (vb - va), 0},
                                         {ca * (ab - aa), sa * (ab - aa), 0}, R);
        REQUIRE(rot.has_value());
        CHECK(*rot == doctest::Approx(t).epsilon(1e-13));
    }

    TEST_CASE("closing estimate: none when it never closes, zero when touching, edge cases")
    {
        // receding with no acceleration, or no motion
        CHECK(!closingTime(100.0, -5.0, 0.0).has_value());
        CHECK(!closingTime(100.0, 0.0, 0.0).has_value());
        // receding but accelerating toward: t^2 - 5 t - 100 = 0 has one positive root
        {
            const auto c = closingTime(100.0, -5.0, 2.0);
            REQUIRE(c.has_value());
            CHECK(*c == doctest::Approx(0.5 * (5.0 + std::sqrt(25.0 + 400.0))).epsilon(1e-14));
        }
        // closing but decelerating to a stop short of the gap: v = 5, a = -2 stops after 6.25 m of 100
        CHECK(!closingTime(100.0, 5.0, -2.0).has_value());
        // closing, decelerating, still reaches: 5 t - t^2 = 4 has roots 1 and 4; the first counts
        {
            const auto c = closingTime(4.0, 5.0, -2.0);
            REQUIRE(c.has_value());
            CHECK(*c == doctest::Approx(1.0).epsilon(1e-14));
        }
        // stops exactly at the gap (discriminant 0): t = 2.5
        {
            const auto c = closingTime(6.25, 5.0, -2.0);
            REQUIRE(c.has_value());
            CHECK(*c == doctest::Approx(2.5).epsilon(1e-12));
        }
        // already touching or overlapping
        CHECK(*closingTime(0.0, -1.0, 0.0) == 0.0);
        CHECK(*closingTime(-3.0, -1.0, -1.0) == 0.0);
        CHECK(*estimateClosing({3, 0, 0}, {1, 0, 0}, {0, 0, 0}, 4.0) == 0.0);
        // non-finite
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(!closingTime(nan, 1.0, 0.0).has_value());
        CHECK(!closingTime(1.0, nan, 0.0).has_value());
        // tiny acceleration: the cancellation-free root equals the no-acceleration answer
        {
            const auto c = closingTime(100.0, 5.0, 1e-300);
            REQUIRE(c.has_value());
            CHECK(*c == doctest::Approx(20.0).epsilon(1e-14));
            const auto c2 = closingTime(100.0, 5.0, 1e-20);
            REQUIRE(c2.has_value());
            CHECK(*c2 == doctest::Approx(20.0).epsilon(1e-12));
        }
        // purely sideways motion never closes along the line of centres
        CHECK(!estimateClosing({10, 0, 0}, {0, 3, 0}, {0, 0, 0}, 1.0).has_value());
    }

    TEST_CASE("look-ahead: 3 s of real time at the current warp")
    {
        CHECK(lookAheadTime(1.0) == 3.0);
        CHECK(lookAheadTime(1e6) == 3e6);
        CHECK(lookAheadTime(0.0) == 0.0);
        CHECK(lookAheadTime(-5.0) == 0.0);

        CHECK(withinLookAhead(2.9e5, 1e5).value() == 2.9e5);
        CHECK(withinLookAhead(3e5, 1e5).value() == 3e5); // the boundary counts
        CHECK(!withinLookAhead(3.0000001e5, 1e5).has_value());
        CHECK(withinLookAhead(0.0, 10.0).value() == 0.0);
        CHECK(!withinLookAhead(-1.0, 10.0).has_value());
        CHECK(!withinLookAhead(std::numeric_limits<double>::infinity(), 1e6).has_value());
        CHECK(!withinLookAhead(1.0, 0.0).has_value()); // warp 0: only t = 0 is within the (empty) window
        CHECK(withinLookAhead(0.0, 0.0).has_value());

        // Through a real prediction: a radial drop from 20 to 1 (mu = 1) takes sqrt(20^3 / 2) (acos(sqrt(1/20)) + sqrt(19/400)) = 98.87 s
        const RelState rs{{20, 0, 0}, {0, 0, 0}};
        const ImpactPrediction pr = predictParentImpact(rs.r, rs.v, 1.0, 1.0);
        REQUIRE(pr.kind == ImpactKind::Impact);
        CHECK(pr.t == doctest::Approx(std::sqrt(4000.0) * (std::acos(std::sqrt(0.05)) + std::sqrt(0.05 * 0.95))).epsilon(1e-9));
        CHECK(!impactWithinLookAhead(pr, 10.0).has_value()); // look-ahead 30 s
        CHECK(!impactWithinLookAhead(pr, 32.0).has_value()); // 96 s
        CHECK(impactWithinLookAhead(pr, 33.0).has_value());  // 99 s
        CHECK(impactWithinLookAhead(pr, 1e6).value() == pr.t);
        // the boundary is exactly t = 3 warp
        CHECK(!impactWithinLookAhead(pr, pr.t / 3.0 * 0.99).has_value());
        CHECK(impactWithinLookAhead(pr, pr.t / 3.0 * 1.01).has_value());
        // other kinds report nothing
        ImpactPrediction none;
        CHECK(!impactWithinLookAhead(none, 1e6).has_value());
        CHECK(!impactWithinLookAhead(none, 1e6, true).has_value());
    }
}
