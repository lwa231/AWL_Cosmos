#include <doctest.h>
#include "core/bodies.h"

using namespace cosmos;

namespace {
BodyDesc make(const char* n, double mu, double x)
{
    BodyDesc d;
    d.name = n;
    d.mu = mu;
    d.radius = mu * 0.5;
    d.position = {x, x + 1.0, x + 2.0};
    d.velocity = {-x, -x - 1.0, -x - 2.0};
    return d;
}
}

TEST_SUITE("bodies")
{
    TEST_CASE("empty store")
    {
        Bodies b;
        CHECK(b.size() == 0);
        CHECK(b.consistent());
    }

    TEST_CASE("fields read back by index")
    {
        Bodies b;
        BodyDesc d;
        d.name = "Domus";
        d.kind = BodyKind::Planet;
        d.kindLocked = true;
        d.fictionalDensity = true;
        d.mu = 3.767e12;
        d.radius = 6.2e5;
        d.spinRate = 1.25e-4;
        d.inertiaFactor = 0.33;
        d.atmosphere = Atmosphere{1.22, 7000.0};
        d.colour = {0.28f, 0.48f, 0.78f};
        d.position = {1.0, 2.0, 3.0};
        d.velocity = {4.0, 5.0, 6.0};
        BodyId id = b.add(d);

        REQUIRE(b.size() == 1);
        CHECK(b.ids[0] == id);
        CHECK(b.name[0] == "Domus");
        CHECK(b.kind[0] == BodyKind::Planet);
        CHECK(b.kindLocked[0] == 1);
        CHECK(b.fictionalDensity[0] == 1);
        CHECK(b.mu[0] == 3.767e12);
        CHECK(b.radius[0] == 6.2e5);
        CHECK(b.spinRate[0] == 1.25e-4);
        CHECK(b.inertiaFactor[0] == 0.33);
        REQUIRE(b.atmosphere[0].has_value());
        CHECK(b.atmosphere[0]->rho0 == 1.22);
        CHECK(b.atmosphere[0]->scaleHeight == 7000.0);
        CHECK(b.colour[0][2] == 0.78f);
        CHECK(b.state.x[0] == 1.0); CHECK(b.state.y[0] == 2.0); CHECK(b.state.z[0] == 3.0);
        CHECK(b.state.vx[0] == 4.0); CHECK(b.state.vy[0] == 5.0); CHECK(b.state.vz[0] == 6.0);
        CHECK(b.consistent());
    }

    TEST_CASE("defaults: no atmosphere, inertia factor 0.4")
    {
        Bodies b;
        b.add(BodyDesc{});
        CHECK_FALSE(b.atmosphere[0].has_value());
        CHECK(b.inertiaFactor[0] == 0.4);
    }

    TEST_CASE("ids are unique and resolve to their index")
    {
        Bodies b;
        BodyId a = b.add(make("A", 1, 10));
        BodyId c = b.add(make("B", 2, 20));
        BodyId d = b.add(make("C", 3, 30));
        CHECK(a != c);
        CHECK(c != d);
        CHECK(a != d);
        CHECK(b.find(a).value() == 0);
        CHECK(b.find(c).value() == 1);
        CHECK(b.find(d).value() == 2);
        CHECK(b.consistent());
    }

    TEST_CASE("remove from the middle keeps order and values")
    {
        Bodies b;
        BodyId a = b.add(make("A", 1, 10));
        BodyId m = b.add(make("B", 2, 20));
        BodyId c = b.add(make("C", 3, 30));

        CHECK(b.remove(m));
        REQUIRE(b.size() == 2);
        CHECK(b.ids[0] == a);
        CHECK(b.ids[1] == c);
        CHECK(b.name[0] == "A");
        CHECK(b.name[1] == "C");
        CHECK(b.mu[0] == 1.0);
        CHECK(b.mu[1] == 3.0);
        CHECK(b.state.x[0] == 10.0);
        CHECK(b.state.x[1] == 30.0);
        CHECK(b.state.vz[1] == -32.0);
        CHECK(b.find(a).value() == 0);
        CHECK(b.find(c).value() == 1);
        CHECK_FALSE(b.find(m).has_value());
        CHECK(b.consistent());
    }

    TEST_CASE("remove first and last")
    {
        Bodies b;
        BodyId a = b.add(make("A", 1, 10));
        BodyId m = b.add(make("B", 2, 20));
        BodyId c = b.add(make("C", 3, 30));

        CHECK(b.remove(a));
        CHECK(b.ids[0] == m);
        CHECK(b.consistent());
        CHECK(b.remove(c));
        REQUIRE(b.size() == 1);
        CHECK(b.ids[0] == m);
        CHECK(b.find(m).value() == 0);
        CHECK(b.remove(m));
        CHECK(b.size() == 0);
        CHECK(b.consistent());
    }

    TEST_CASE("removed ids are never reused")
    {
        Bodies b;
        BodyId a = b.add(make("A", 1, 10));
        BodyId c = b.add(make("B", 2, 20));
        b.remove(c);
        BodyId d = b.add(make("C", 3, 30));
        CHECK(d != a);
        CHECK(d != c);
        CHECK(d > c);
        b.remove(a);
        BodyId e = b.add(make("D", 4, 40));
        CHECK(e > d);
    }

    TEST_CASE("removing an unknown id changes nothing")
    {
        Bodies b;
        b.add(make("A", 1, 10));
        CHECK_FALSE(b.remove(9999));
        CHECK(b.size() == 1);
        CHECK(b.consistent());
    }
}
