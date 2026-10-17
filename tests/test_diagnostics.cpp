#include <doctest.h>
#include "core/bodies.h"
#include "core/diagnostics.h"

using namespace cosmos;

namespace {
BodyDesc body(double mu, double x, double y, double vx, double vy)
{
    BodyDesc d;
    d.mu = mu;
    d.position = {x, y, 0.0};
    d.velocity = {vx, vy, 0.0};
    return d;
}
}

TEST_SUITE("diagnostics")
{
    TEST_CASE("empty store is all zeros")
    {
        Bodies b;
        const Conserved c = conserved(b);
        CHECK(c.energyG == 0.0);
        CHECK(c.momentumG.x == 0.0);
        CHECK(c.angularMomentumG.z == 0.0);
    }

    TEST_CASE("equal-mass circular binary has known values")
    {
        Bodies b;
        b.add(body(0.5, -0.5, 0, 0, -0.5));
        b.add(body(0.5, 0.5, 0, 0, 0.5));
        const Conserved c = conserved(b);
        CHECK(c.energyG == -0.125);
        CHECK(c.momentumG.x == 0.0);
        CHECK(c.momentumG.y == 0.0);
        CHECK(c.angularMomentumG.z == 0.25);
    }

    TEST_CASE("momentum is the mu-weighted velocity sum")
    {
        Bodies b;
        BodyDesc d;
        d.mu = 2.0;
        d.velocity = {1.0, 2.0, 3.0};
        b.add(d);
        const Conserved c = conserved(b);
        CHECK(c.momentumG.x == 2.0);
        CHECK(c.momentumG.y == 4.0);
        CHECK(c.momentumG.z == 6.0);
        CHECK(c.energyG == 14.0);
    }

    TEST_CASE("spin adds k mu R^2 omega to Lz")
    {
        Bodies b;
        BodyDesc d;
        d.mu = 5.0;
        d.radius = 2.0;
        d.spinRate = 3.0;
        d.inertiaFactor = 0.4;
        b.add(d);
        const Conserved c = conserved(b);
        CHECK(c.angularMomentumG.z == doctest::Approx(24.0).epsilon(1e-14).scale(0));
        CHECK(c.angularMomentumG.x == 0.0);
        CHECK(c.angularMomentumG.y == 0.0);
    }

    TEST_CASE("potential energy is -mu_i mu_j / r")
    {
        Bodies b;
        b.add(body(2.0, 0, 0, 0, 0));
        b.add(body(3.0, 4, 0, 0, 0));
        CHECK(conserved(b).energyG == -1.5);
    }
}
