#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "core/bodies.h"
#include "core/diagnostics.h"
#include "core/gravity.h"

using namespace cosmos;

namespace {
BodyDesc body(double mu, double radius, double x, double y, double z = 0.0, double vx = 0.0, double vy = 0.0)
{
    BodyDesc d;
    d.mu = mu;
    d.radius = radius;
    d.position = {x, y, z};
    d.velocity = {vx, vy, 0.0};
    return d;
}

// Five bodies with awkward, non-round numbers, none touching.
Bodies scatter(double radiusScale)
{
    Bodies b;
    b.add(body(1.3, 0.11 * radiusScale, 0.1, -0.4, 0.7));
    b.add(body(0.8, 0.07 * radiusScale, 2.2, 1.1, -0.3));
    b.add(body(2.5, 0.19 * radiusScale, -1.7, 0.9, 0.2));
    b.add(body(0.4, 0.03 * radiusScale, 0.6, -2.3, 1.5));
    b.add(body(1.1, 0.13 * radiusScale, -0.8, -0.1, -1.9));
    return b;
}
}

TEST_SUITE("overlap")
{
    TEST_CASE("radius 0 everywhere reproduces the point-mass function bit for bit")
    {
        const Bodies b = scatter(0.0);
        Accel ref, ov;
        computeAccelerations(b.state, b.mu, ref);
        computeAccelerations(b.state, b.mu, b.radius, ov);
        for (std::size_t i = 0; i < b.size(); ++i) {
            CHECK(ov.ax[i] == ref.ax[i]);
            CHECK(ov.ay[i] == ref.ay[i]);
            CHECK(ov.az[i] == ref.az[i]);
        }
    }

    TEST_CASE("non-overlapping finite radii also reproduce the point-mass function bit for bit")
    {
        const Bodies b = scatter(1.0); // radii <= 0.19, nearest pair > 1.5 apart
        Accel ref, ov;
        computeAccelerations(b.state, b.mu, ref);
        computeAccelerations(b.state, b.mu, b.radius, ov);
        for (std::size_t i = 0; i < b.size(); ++i) {
            CHECK(ov.ax[i] == ref.ax[i]);
            CHECK(ov.ay[i] == ref.ay[i]);
            CHECK(ov.az[i] == ref.az[i]);
        }
    }

    TEST_CASE("inside R_c the field is mu d / R_c^3 with R_c the larger radius")
    {
        Bodies b;
        b.add(body(1.0, 0.5, 0, 0, 0));
        b.add(body(2.0, 2.0, 1.0, 0, 0)); // r = 1 < R_c = 2
        Accel a;
        computeAccelerations(b.state, b.mu, b.radius, a);
        CHECK(a.ax[0] == doctest::Approx(2.0 * 1.0 / 8.0).epsilon(1e-15).scale(0));
        CHECK(a.ax[1] == doctest::Approx(-1.0 * 1.0 / 8.0).epsilon(1e-15).scale(0));
        CHECK(a.ay[0] == 0.0);
        CHECK(a.az[1] == 0.0);
    }

    TEST_CASE("inside R_c the field is linear in separation, and zero at coincidence")
    {
        Bodies b1, b2, b3;
        b1.add(body(3.0, 2.0, 0, 0, 0));
        b1.add(body(1.0, 0.0, 1.0, 0, 0));
        b2.add(body(3.0, 2.0, 0, 0, 0));
        b2.add(body(1.0, 0.0, 0.5, 0, 0));
        b3.add(body(3.0, 2.0, 0, 0, 0));
        b3.add(body(1.0, 0.0, 0, 0, 0));
        Accel a1, a2, a3;
        computeAccelerations(b1.state, b1.mu, b1.radius, a1);
        computeAccelerations(b2.state, b2.mu, b2.radius, a2);
        computeAccelerations(b3.state, b3.mu, b3.radius, a3);
        CHECK(a1.ax[1] / a2.ax[1] == doctest::Approx(2.0).epsilon(1e-15).scale(0));
        CHECK(a3.ax[0] == 0.0);
        CHECK(a3.ax[1] == 0.0);
    }

    TEST_CASE("continuous at r = R_c")
    {
        const double R = 1.7, mu = 2.3;
        const double eps = 1e-9;
        double aIn, aOn, aOut;
        for (int k = 0; k < 3; ++k) {
            const double r = k == 0 ? R * (1.0 - eps) : k == 1 ? R : R * (1.0 + eps);
            Bodies b;
            b.add(body(mu, R, 0, 0, 0));
            b.add(body(0.0, 0.0, r * 0.6, r * 0.8, 0.0));
            Accel a;
            computeAccelerations(b.state, b.mu, b.radius, a);
            const double mag = std::sqrt(a.ax[1] * a.ax[1] + a.ay[1] * a.ay[1]);
            (k == 0 ? aIn : k == 1 ? aOn : aOut) = mag;
        }
        const double exact = mu / (R * R);
        // The outer side at r = R is the exterior formula, the inner side the interior one: equal to rounding.
        CHECK(aOn == doctest::Approx(exact).epsilon(1e-14).scale(0));
        // One side of the edge to the other differs only by the 2e-9 change in r (field changes by 2 eps x).
        CHECK(std::abs(aIn - aOut) <= 5.0 * eps * exact);
        CHECK(aIn == doctest::Approx(exact).epsilon(2.0 * eps).scale(0));
        CHECK(aOut == doctest::Approx(exact).epsilon(3.0 * eps).scale(0));
    }

    TEST_CASE("Newton's third law holds inside overlaps: sum mu_i a_i = 0")
    {
        Bodies b = scatter(20.0); // radii up to 3.8: most pairs overlap
        Accel a;
        computeAccelerations(b.state, b.mu, b.radius, a);
        double px = 0, py = 0, pz = 0, scale = 0;
        for (std::size_t i = 0; i < b.size(); ++i) {
            px += b.mu[i] * a.ax[i];
            py += b.mu[i] * a.ay[i];
            pz += b.mu[i] * a.az[i];
            scale += b.mu[i] * (std::abs(a.ax[i]) + std::abs(a.ay[i]) + std::abs(a.az[i]));
        }
        CHECK(std::abs(px) <= 1e-14 * scale);
        CHECK(std::abs(py) <= 1e-14 * scale);
        CHECK(std::abs(pz) <= 1e-14 * scale);
        // and the overlap really changed something
        Accel pt;
        computeAccelerations(b.state, b.mu, pt);
        CHECK(a.ax[0] != pt.ax[0]);
    }

    TEST_CASE("pair potential is continuous at R_c and matches the interior formula")
    {
        const double mi = 1.5, mj = 0.5, R = 2.0;
        CHECK(pairPotentialG(mi, mj, R, R) == doctest::Approx(-mi * mj / R).epsilon(1e-15).scale(0));
        const double eps = 1e-9;
        CHECK(pairPotentialG(mi, mj, R * (1 - eps), R) == doctest::Approx(-mi * mj / R).epsilon(2e-9).scale(0));
        // centre of the uniform sphere: -1.5 mu / R
        CHECK(pairPotentialG(mi, mj, 0.0, R) == doctest::Approx(-1.5 * mi * mj / R).epsilon(1e-15).scale(0));
        // radius 0 is the point-mass value exactly
        CHECK(pairPotentialG(mi, mj, 3.0, 0.0) == -(mi * mj / 3.0));
    }

    TEST_CASE("the potential is the integral of the force: -dU/dr = mu_i |a_i|")
    {
        const double mi = 1.5, mj = 0.5, R = 2.0;
        for (double r : {0.3, 1.0, 1.9, 2.5, 4.0}) {
            const double h = 1e-6;
            const double dUdr = (pairPotentialG(mi, mj, r + h, R) - pairPotentialG(mi, mj, r - h, R)) / (2.0 * h);
            Bodies b;
            b.add(body(mi, R, 0, 0, 0));
            b.add(body(mj, 0.0, r, 0, 0));
            Accel a;
            computeAccelerations(b.state, b.mu, b.radius, a);
            // body 0 is pulled toward +x with force mu_i a_0 = dU/dr
            CHECK(mi * a.ax[0] == doctest::Approx(dUdr).epsilon(1e-8).scale(0));
        }
    }

    TEST_CASE("conserved() energy uses the interior potential for overlapping pairs and is unchanged otherwise")
    {
        Bodies over;
        over.add(body(2.0, 1.0, 0, 0, 0));
        over.add(body(3.0, 0.5, 0.5, 0, 0));
        // R_c = 1, r = 0.5: U = -6 (3 - 0.25) / 2 = -8.25
        CHECK(conserved(over).energyG == doctest::Approx(-8.25).epsilon(1e-15).scale(0));
        CHECK(energyG(over) == conserved(over).energyG);

        Bodies apart;
        apart.add(body(2.0, 1.0, 0, 0, 0));
        apart.add(body(3.0, 0.5, 4, 0, 0));
        CHECK(conserved(apart).energyG == -1.5); // point-mass value, as before
    }

    TEST_CASE("energy is conserved by KDK through an overlap passage (force and potential agree)")
    {
        // Two equal bodies fall through each other: R = 1, mu = 1, start 3 apart, at rest.
        auto run = [](double dt) {
            Bodies b;
            b.add(body(1.0, 1.0, -1.5, 0, 0));
            b.add(body(1.0, 1.0, 1.5, 0, 0));
            const double e0 = conserved(b).energyG;
            Accel a;
            computeAccelerations(b.state, b.mu, b.radius, a);
            double worst = 0.0;
            const double T = 12.0;
            const long steps = static_cast<long>(T / dt);
            bool overlapped = false;
            for (long k = 0; k < steps; ++k) {
                BodyState& s = b.state;
                for (std::size_t i = 0; i < 2; ++i) { s.vx[i] += a.ax[i] * 0.5 * dt; s.vy[i] += a.ay[i] * 0.5 * dt; }
                for (std::size_t i = 0; i < 2; ++i) { s.x[i] += s.vx[i] * dt; s.y[i] += s.vy[i] * dt; }
                computeAccelerations(s, b.mu, b.radius, a);
                for (std::size_t i = 0; i < 2; ++i) { s.vx[i] += a.ax[i] * 0.5 * dt; s.vy[i] += a.ay[i] * 0.5 * dt; }
                if (std::abs(s.x[1] - s.x[0]) < 1.0) overlapped = true;
                worst = std::max(worst, std::abs(conserved(b).energyG - e0) / std::abs(e0));
            }
            CHECK(overlapped);
            return worst;
        };
        const double e1 = run(1e-2);
        const double e2 = run(5e-3);
        // Measured: max relative drift 1.8e-4 at dt = 0.01 and 4.7e-5 at 0.005 (ratio 3.8, second order).
        CHECK(e1 < 3e-4);
        CHECK(e2 < 8e-5);
        CHECK(e1 / e2 == doctest::Approx(4.0).epsilon(0.3));
    }
}
