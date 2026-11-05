#include <doctest.h>
#include <cmath>
#include "app/drag_place.h"

using namespace cosmos;
using namespace cosmos::app;

namespace {
// Sun-like star, an Earth-like planet at 1 AU, a Moon-like moon 3.84e8 m from it.
std::vector<PlaceBody> system3()
{
    const double muS = 1.327e20, muE = 3.986e14, muM = 4.9e12, au = 1.496e11, dm = 3.844e8;
    const double vE = std::sqrt(muS / au), vM = std::sqrt(muE / dm);
    return {{1, {0, 0, 0}, {0, 0, 0}, muS, 6.96e8},
            {2, {au, 0, 0}, {0, vE, 0}, muE, 6.371e6},
            {3, {au + dm, 0, 0}, {0, vE + vM, 0}, muM, 1.737e6}};
}
} // namespace

TEST_CASE("drag place: parent is the smallest sphere containing the point")
{
    const auto b = system3();
    const double au = 1.496e11, dm = 3.844e8;
    CHECK(snapPlacement(b, {au + dm + 1.0e7, 0, 0}).parentId == 3);   // inside the moon's Hill sphere (~6e7 m)
    CHECK(snapPlacement(b, {au + 1.0e8, 0, 0}).parentId == 2);        // near the planet, outside the moon's sphere
    CHECK(snapPlacement(b, {0.5 * au, 0, 0}).parentId == 1);          // in the star's domain
    // The dragged body never orbits itself: excluding the moon, the same point snaps to the planet
    CHECK(snapPlacement(b, {au + dm + 1.0e7, 0, 0}, 0.0, 3).parentId == 2);
}

TEST_CASE("drag place: circular velocity is prograde, perpendicular and of circular speed")
{
    const auto b = system3();
    const double au = 1.496e11, muE = 3.986e14;
    const Vec3 p{au, 4.0e7, 0};
    const PlaceSnap s = snapPlacement(b, p, 0.0);
    REQUIRE(s.parentId == 2);
    CHECK(s.distance == doctest::Approx(4.0e7));
    CHECK(s.circularSpeed == doctest::Approx(std::sqrt(muE / 4.0e7)));
    const Vec3 rel = s.velocity - s.parentV;
    CHECK(norm(rel) == doctest::Approx(s.circularSpeed));
    CHECK(std::fabs(dot(rel, p - s.parentR)) < 1e-6 * norm(rel) * s.distance);
    CHECK(cross(p - s.parentR, rel).z > 0.0);   // counter-clockwise
    const PlaceSnap r = snapPlacement(b, p, 0.0, 0, true);
    CHECK(cross(p - r.parentR, r.velocity - r.parentV).z < 0.0);
    CHECK_FALSE(s.insideParent);
    CHECK(snapPlacement(b, {au + 1.0e6, 0, 0}).insideParent);
}

TEST_CASE("drag place: rings, arrow velocity and preview horizon")
{
    const auto b = system3();
    const auto rings = placementRings(b);
    REQUIRE(rings.size() == 3);
    CHECK(std::isinf(rings[0].hill));          // the star dominates
    CHECK(rings[1].hill > 1.0e9);              // Earth's Hill sphere ~1.5e9 m
    CHECK(rings[1].hill < 1.6e9);
    CHECK(rings[1].soi == doctest::Approx(1.496e11 * std::pow((3.986e14 + 4.9e12) / 1.327e20, 0.4)).epsilon(0.01));   // the Earth-Moon pair
    CHECK(rings[2].soi < rings[1].soi);
    CHECK(placementRings(b, 2).size() == 2);

    const PlaceSnap s = snapPlacement(b, {1.496e11, 4.0e7, 0});
    const double k = arrowScaleFor(s, 1.0e6);
    CHECK(k * s.circularSpeed == doctest::Approx(1.0e6));
    const Vec3 v = velocityFromArrow(s, {2.0e6, 0, 0}, k);
    CHECK(norm(v - s.parentV) == doctest::Approx(2.0 * s.circularSpeed));
    const double period = 2.0 * 3.14159265358979323846 * s.distance / s.circularSpeed;
    CHECK(previewHorizon(s, 2.0) == doctest::Approx(2.0 * period));
    CHECK(snapPlacement({}, {1, 2, 0}).parentId == 0);
    CHECK(previewHorizon(snapPlacement({}, {1, 2, 0})) == 86400.0);
}
