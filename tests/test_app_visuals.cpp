#include <doctest.h>
#include <cmath>
#include <vector>
#include "app/visuals.h"

using namespace cosmos;
using namespace cosmos::app;

// V1.1 step 3 helpers: temperature <-> colour, the detail pick and the accretion-disk normal.

TEST_CASE("visuals: blackbody colours and the temperature of an authored star colour")
{
    const RenderColour cool = blackbodyColour(3000.0f), sun = blackbodyColour(5800.0f), hot = blackbodyColour(20000.0f);
    CHECK(cool.r == doctest::Approx(1.0f));
    CHECK(cool.b < 0.5f);                 // red-orange
    CHECK(hot.b == doctest::Approx(1.0f));
    CHECK(hot.r < 0.7f);                  // blue-white
    CHECK(sun.g > cool.g);
    CHECK(blackbodyColour(100.0f).r == doctest::Approx(1.0f));        // clamped below the table
    CHECK(blackbodyColour(1e7f).b == doctest::Approx(1.0f));          // and above it

    // Round trip: the colour of a temperature gives that temperature back (1 % search steps)
    for (float k : {2000.0f, 3500.0f, 5000.0f, 9000.0f, 15000.0f}) CHECK(temperatureFromColour(blackbodyColour(k)) == doctest::Approx(k).epsilon(0.03));
    // Lucerna's pale yellow (1, 0.95, 0.82) is a sun-like star; white is about 6500 K
    const float lucerna = temperatureFromColour({1.0f, 0.95f, 0.82f});
    CHECK(lucerna > 4500.0f);
    CHECK(lucerna < 6000.0f);
    CHECK(temperatureFromColour({1.0f, 1.0f, 1.0f}) == doctest::Approx(6500.0f).epsilon(0.05));
    CHECK(starTemperature(BodyKind::NeutronStar, {1, 1, 1}) > 1e5f);
    CHECK(starTemperature(BodyKind::Planet, {1, 1, 1}) == 0.0f);
}

TEST_CASE("visuals: the most massive planet-class body in view gets the detail")
{
    std::vector<DisplayBody> bodies(4);
    std::vector<RenderBody> rendered(4);
    bodies[0].kind = BodyKind::Star;     bodies[0].mu = 1e20; rendered[0].radius = 1e9f;
    bodies[1].kind = BodyKind::GasGiant; bodies[1].mu = 1e17; rendered[1].radius = 7e7f; rendered[1].x = 1e12f;   // off-screen
    bodies[2].kind = BodyKind::Planet;   bodies[2].mu = 4e14; rendered[2].radius = 6e6f; rendered[2].x = 1e8f;
    bodies[3].kind = BodyKind::Moon;     bodies[3].mu = 5e12; rendered[3].radius = 1.7e6f; rendered[3].x = -1e8f;
    CHECK(pickDetailBody(bodies, rendered, 1e6f, 1000, 800) == 2);    // the star is not planet-class, the giant is not in view
    CHECK(pickDetailBody(bodies, rendered, 1e7f, 1000, 800) == -1);   // 0.6 px: too small to show detail
    rendered[1].x = 3e8f;
    CHECK(pickDetailBody(bodies, rendered, 1e6f, 1000, 800) == 1);
}

TEST_CASE("visuals: accretion-disk normal follows the angular momentum of what orbits the hole")
{
    std::vector<DisplayBody> bodies(3);
    bodies[0].kind = BodyKind::BlackHole; bodies[0].mu = 1e21;
    float n[3];
    diskNormal(bodies, 0, n);   // nothing moves: face-on default
    CHECK(n[2] == doctest::Approx(1.0f));

    bodies[1].mu = 1e12; bodies[1].r = {1e9, 0, 0}; bodies[1].v = {0, -1e5, 0};       // clockwise in the plane: -z
    bodies[2].mu = 1e6;  bodies[2].r = {0, 1e11, 0}; bodies[2].v = {0, 0, 1e4};       // far and light, tilted: barely counts
    diskNormal(bodies, 0, n);
    CHECK(n[2] < -0.99f);
    CHECK(std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) == doctest::Approx(1.0f));

    bodies[1].v = {0, 0, 1e5};   // an orbit over the poles: normal in the plane (r x v = -y)
    diskNormal(bodies, 0, n);
    CHECK(n[1] < -0.99f);
}
