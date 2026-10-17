#include <doctest.h>
#include <cmath>
#include "core/bodies.h"
#include "core/gravity.h"

using namespace cosmos;

namespace {
BodyDesc body(double mu, double x, double y, double z)
{
    BodyDesc d;
    d.mu = mu;
    d.position = {x, y, z};
    return d;
}
}

TEST_SUITE("gravity")
{
    TEST_CASE("two bodies: exact accelerations toward each other")
    {
        Bodies b;
        b.add(body(1.0, 0, 0, 0));
        b.add(body(2.0, 2, 0, 0));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        CHECK(a.ax[0] == 0.5);
        CHECK(a.ax[1] == -0.25);
        CHECK(a.ay[0] == 0.0);
        CHECK(a.az[1] == 0.0);
    }

    TEST_CASE("acceleration scales as 1/r^2")
    {
        Bodies near, far;
        near.add(body(3.0, 0, 0, 0));
        near.add(body(0.0, 1, 0, 0));
        far.add(body(3.0, 0, 0, 0));
        far.add(body(0.0, 2, 0, 0));
        Accel an, af;
        computeAccelerations(near.state, near.mu, an);
        computeAccelerations(far.state, far.mu, af);
        CHECK(an.ax[1] / af.ax[1] == doctest::Approx(4.0).epsilon(1e-14).scale(0));
    }

    TEST_CASE("a body with mu = 0 exerts no force but feels one")
    {
        Bodies b;
        b.add(body(5.0, 0, 0, 0));
        b.add(body(0.0, 0, 2, 0));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        CHECK(a.ax[0] == 0.0);
        CHECK(a.ay[0] == 0.0);
        CHECK(a.ay[1] == doctest::Approx(-1.25).epsilon(1e-14).scale(0));
    }

    TEST_CASE("force points along the line of centres in 3D")
    {
        Bodies b;
        b.add(body(1.0, 0, 0, 0));
        b.add(body(1.0, 1, 2, 2));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        // distance 3, |a| = 1/9, direction (1,2,2)/3
        CHECK(a.ax[0] == doctest::Approx(1.0 / 27.0).epsilon(1e-14).scale(0));
        CHECK(a.ay[0] == doctest::Approx(2.0 / 27.0).epsilon(1e-14).scale(0));
        CHECK(a.az[0] == doctest::Approx(2.0 / 27.0).epsilon(1e-14).scale(0));
        CHECK(a.ax[1] == -a.ax[0]);
    }

    TEST_CASE("total momentum change is zero: sum mu_i a_i = 0")
    {
        Bodies b;
        b.add(body(1.3, 0.1, -0.4, 0.7));
        b.add(body(0.8, 2.2, 1.1, -0.3));
        b.add(body(2.5, -1.7, 0.9, 0.2));
        b.add(body(0.4, 0.6, -2.3, 1.5));
        b.add(body(1.1, -0.8, -0.1, -1.9));
        Accel a;
        computeAccelerations(b.state, b.mu, a);
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
    }

    TEST_CASE("output is overwritten and resized")
    {
        Bodies b;
        b.add(body(1.0, 0, 0, 0));
        b.add(body(1.0, 1, 0, 0));
        Accel a;
        a.ax.assign(7, 99.0);
        a.ay.assign(7, 99.0);
        a.az.assign(7, 99.0);
        computeAccelerations(b.state, b.mu, a);
        REQUIRE(a.ax.size() == 2);
        CHECK(a.ax[0] == 1.0);
        CHECK(a.ay[0] == 0.0);
    }

    TEST_CASE("empty and single-body systems")
    {
        Bodies b;
        Accel a;
        computeAccelerations(b.state, b.mu, a);
        CHECK(a.ax.empty());
        b.add(body(1.0, 0, 0, 0));
        computeAccelerations(b.state, b.mu, a);
        REQUIRE(a.ax.size() == 1);
        CHECK(a.ax[0] == 0.0);
    }
}
