#include <doctest.h>
#include <cmath>
#include "app/editor_model.h"
#include "app/sim_driver.h"
#include "test_app_editor_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace editor_test;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// The sun and an Earth-like planet, the planet placed with "orbit around X at distance d"
struct SunAndPlanet {
    EditorModel m;
    std::uint64_t star = 0, planet = 0;
    double d = 1.495978707e11;
    SunAndPlanet()
    {
        star = m.addBody(BodyKind::Star).id;
        planet = m.addBody(BodyKind::Planet).id;
        REQUIRE(m.setOrbitAround(planet, star, d).ok);
    }
    double muTotal() const { return m.body(star)->mu + m.body(planet)->mu; }
};

} // namespace

// "Orbit around X at distance d" uses the Jacobi rule of plan 5.9 through resolveScene.
TEST_SUITE("app editor orbit")
{
    TEST_CASE("the placed body is at distance d with the circular speed for mu_parent + mu_body")
    {
        SunAndPlanet s;
        Vec3 rs, vs, rp, vp;
        REQUIRE(s.m.resolvedState(s.star, rs, vs));
        REQUIRE(s.m.resolvedState(s.planet, rp, vp));
        const Vec3 r = rp - rs, v = vp - vs;
        CHECK(norm(r) == doctest::Approx(s.d).epsilon(1e-12));
        CHECK(norm(v) == doctest::Approx(std::sqrt(s.muTotal() / s.d)).epsilon(1e-12));
        CHECK(dot(r, v) == doctest::Approx(0.0).epsilon(1e-6 * norm(r) * norm(v)));   // circular: perpendicular
        CHECK((r.x * v.y - r.y * v.x) > 0.0);                                         // prograde: counter-clockwise
        // The system is at its barycentre with zero momentum (5.9 step 4)
        const double mu1 = s.m.body(s.star)->mu, mu2 = s.m.body(s.planet)->mu;
        const Vec3 bary = (mu1 * rs + mu2 * rp) / (mu1 + mu2);
        const Vec3 mom = (mu1 * vs + mu2 * vp) / (mu1 + mu2);
        CHECK(norm(bary) < 1e-6 * s.d);
        CHECK(norm(mom) < 1e-9 * norm(v));
    }

    TEST_CASE("simulated for one Kepler period it comes back to where it started")
    {
        SunAndPlanet s;
        ResolvedScene res;
        std::string err;
        REQUIRE(resolveScene(s.m.scene(), res, err, s.m.resolveOptions()));
        SimDriver driver(res, s.m.scene().settings);

        const auto rel = [&](const DisplayState& st) {
            Vec3 a, b;
            for (const DisplayBody& body : st.bodies) {
                if (body.id == res.bodyIds[0]) a = body.r;
                if (body.id == res.bodyIds[1]) b = body.r;
            }
            return b - a;
        };
        const Vec3 start = rel(driver.state());
        const double period = kTwoPi * std::sqrt(s.d * s.d * s.d / s.muTotal());
        CHECK(period == doctest::Approx(3.1558e7).epsilon(1e-3));   // about a year

        driver.advanceTo(0.5 * period);
        const Vec3 half = rel(driver.state());
        CHECK(norm(half + start) < 1e-6 * s.d);   // opposite side after half a period
        driver.advanceTo(period);
        const Vec3 full = rel(driver.state());
        CHECK(driver.state().t == doctest::Approx(period).epsilon(1e-12));
        CHECK(norm(full - start) < 1e-6 * s.d);
    }

    TEST_CASE("an eccentric orbit has the right periapsis and period")
    {
        SunAndPlanet s;
        SceneOrbit o = *s.m.body(s.planet)->orbit;
        o.e = 0.3;
        o.meanAnomaly_deg = 0.0;   // at periapsis at t = 0
        o.argPeriapsis_deg = 40.0;
        REQUIRE(s.m.setOrbit(s.planet, o).ok);
        Vec3 rs, vs, rp, vp;
        REQUIRE(s.m.resolvedState(s.star, rs, vs));
        REQUIRE(s.m.resolvedState(s.planet, rp, vp));
        const Vec3 r = rp - rs;
        CHECK(norm(r) == doctest::Approx(s.d * 0.7).epsilon(1e-9));
        CHECK(std::atan2(r.y, r.x) == doctest::Approx(40.0 * kTwoPi / 360.0).epsilon(1e-9));

        ResolvedScene res;
        std::string err;
        REQUIRE(resolveScene(s.m.scene(), res, err, s.m.resolveOptions()));
        SimDriver driver(res, s.m.scene().settings);
        const double period = kTwoPi * std::sqrt(s.d * s.d * s.d / s.muTotal());
        driver.advanceTo(period);
        Vec3 a, b;
        for (const DisplayBody& body : driver.state().bodies) {
            if (body.id == res.bodyIds[0]) a = body.r;
            if (body.id == res.bodyIds[1]) b = body.r;
        }
        CHECK(norm((b - a) - r) < 1e-6 * s.d);
    }

    TEST_CASE("retrograde is clockwise")
    {
        SunAndPlanet s;
        SceneOrbit o = *s.m.body(s.planet)->orbit;
        o.retrograde = true;
        REQUIRE(s.m.setOrbit(s.planet, o).ok);
        Vec3 rs, vs, rp, vp;
        REQUIRE(s.m.resolvedState(s.star, rs, vs));
        REQUIRE(s.m.resolvedState(s.planet, rp, vp));
        const Vec3 r = rp - rs, v = vp - vs;
        CHECK((r.x * v.y - r.y * v.x) < 0.0);
        CHECK(norm(r) == doctest::Approx(s.d).epsilon(1e-12));
    }

    TEST_CASE("moons are placed in Jacobi coordinates, nearest first whatever the order they were added in")
    {
        EditorModel m;
        const std::uint64_t star = m.addBody(BodyKind::Star).id;
        const std::uint64_t planet = m.addBody(BodyKind::Planet).id;
        const std::uint64_t far = m.addBody(BodyKind::Moon).id;
        const std::uint64_t near = m.addBody(BodyKind::Moon).id;
        REQUIRE(m.setOrbitAround(planet, star, 1.0e11).ok);
        REQUIRE(m.setOrbitAround(far, planet, 3.0e8).ok);    // added first, but the outer one
        REQUIRE(m.setOrbitAround(near, planet, 1.0e8).ok);
        REQUIRE(m.setMu(near, 2.0e13).ok);                   // heavy enough for the barycentre shift to show
        REQUIRE(m.setMu(far, 3.0e13).ok);

        Vec3 rP, vP, rN, vN, rF, vF;
        REQUIRE(m.resolvedState(planet, rP, vP));
        REQUIRE(m.resolvedState(near, rN, vN));
        REQUIRE(m.resolvedState(far, rF, vF));
        const double muP = m.body(planet)->mu, muN = m.body(near)->mu, muF = m.body(far)->mu;

        // The inner moon is at its distance from the planet, with the speed for mu_planet + mu_moon
        CHECK(norm(rN - rP) == doctest::Approx(1.0e8).epsilon(1e-10));
        CHECK(norm(vN - vP) == doctest::Approx(std::sqrt((muP + muN) / 1.0e8)).epsilon(1e-10));
        // The outer moon is at its distance from the barycentre of the planet and the inner moon, with mu_planet + mu_inner +
        // mu_outer (plan 5.9 step 2)
        const Vec3 rB = (muP * rP + muN * rN) / (muP + muN);
        const Vec3 vB = (muP * vP + muN * vN) / (muP + muN);
        CHECK(norm(rF - rB) == doctest::Approx(3.0e8).epsilon(1e-10));
        CHECK(norm(vF - vB) == doctest::Approx(std::sqrt((muP + muN + muF) / 3.0e8)).epsilon(1e-10));
        // and not at the planet itself
        CHECK(norm(rF - rP) != doctest::Approx(3.0e8).epsilon(1e-6));
    }

    TEST_CASE("a shuttle orbits its parent with mu_parent")
    {
        EditorModel m;
        m.setScene(lucerna());
        const std::uint64_t domus = idOf(m.scene(), "Domus");
        const std::uint64_t sh = m.addShuttle().id;
        const double d = m.suggestedOrbitDistance(domus, ObjectType::Shuttle);
        const SceneBody* dom = m.body(domus);
        CHECK(d > dom->radius);
        REQUIRE(m.setOrbitAround(sh, domus, d).ok);
        Vec3 rD, vD, rS, vS;
        REQUIRE(m.resolvedState(domus, rD, vD));
        REQUIRE(m.resolvedState(sh, rS, vS));
        CHECK(norm(rS - rD) == doctest::Approx(d).epsilon(1e-12));
        CHECK(norm(vS - vD) == doctest::Approx(std::sqrt(dom->mu / d)).epsilon(1e-12));
        // Above the atmosphere (rho = 1e-6 at the top)
        const double top = dom->atmosphere->scaleHeight * std::log(dom->atmosphere->rho0 / 1.0e-6);
        CHECK(d - dom->radius > top);
        CHECK(m.suggestedOrbitDistance(domus, ObjectType::Body) == 4.0 * dom->radius);
        CHECK(m.suggestedOrbitDistance(9999, ObjectType::Body) == 0.0);
    }

    TEST_CASE("orbit around keeps the other elements when the parent stays")
    {
        SunAndPlanet s;
        SceneOrbit o = *s.m.body(s.planet)->orbit;
        o.e = 0.2;
        o.argPeriapsis_deg = 30.0;
        o.meanAnomaly_deg = 120.0;
        o.retrograde = true;
        REQUIRE(s.m.setOrbit(s.planet, o).ok);
        REQUIRE(s.m.setOrbitAround(s.planet, s.star, 2.0e11).ok);
        const SceneOrbit& n = *s.m.body(s.planet)->orbit;
        CHECK(n.a == 2.0e11);
        CHECK(n.e == 0.2);
        CHECK(n.argPeriapsis_deg == 30.0);
        CHECK(n.meanAnomaly_deg == 120.0);
        CHECK(n.retrograde);
    }

    TEST_CASE("invalid orbits are refused with a message and change nothing")
    {
        SunAndPlanet s;
        const std::uint64_t moon = s.m.addBody(BodyKind::Moon).id;
        const std::uint64_t shuttle = s.m.addShuttle().id;
        REQUIRE(s.m.setOrbitAround(moon, s.planet, 4.0e8).ok);
        const Scene before = s.m.scene();
        const std::uint64_t rev = s.m.revision();

        EditResult r = s.m.setOrbitAround(s.planet, s.planet, 1.0e9);
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("itself") != std::string::npos);

        r = s.m.setOrbitAround(s.star, moon, 1.0e9);   // the star would orbit its own moon's moon: a cycle
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("cycle") != std::string::npos);

        r = s.m.setOrbitAround(s.planet, moon, 1.0e9);   // the planet would orbit its own moon
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("cycle") != std::string::npos);

        r = s.m.setOrbitAround(moon, shuttle, 1.0e9);   // a shuttle is not a parent
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("parent") != std::string::npos);
        r = s.m.setOrbitAround(moon, 4242, 1.0e9);
        CHECK_FALSE(r.ok);

        r = s.m.setOrbitAround(moon, s.planet, 0.0);
        CHECK_FALSE(r.ok);
        r = s.m.setOrbitAround(moon, s.planet, -1.0e9);
        CHECK_FALSE(r.ok);

        SceneOrbit o = *s.m.body(moon)->orbit;
        o.e = 1.0;
        r = s.m.setOrbit(moon, o);
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("eccentricity") != std::string::npos);
        o.e = -0.1;
        CHECK_FALSE(s.m.setOrbit(moon, o).ok);
        o.e = 0.1;
        o.a = 0.0;
        CHECK_FALSE(s.m.setOrbit(moon, o).ok);
        o.a = 1.0e9;
        o.meanAnomaly_deg = std::nan("");
        CHECK_FALSE(s.m.setOrbit(moon, o).ok);

        CHECK(sameScene(s.m.scene(), before));
        CHECK(s.m.revision() == rev);
    }

    TEST_CASE("text fields in orbit mode")
    {
        SunAndPlanet s;
        EditorModel& m = s.m;
        CHECK(m.inOrbitMode(s.planet));
        CHECK_FALSE(m.inOrbitMode(s.star));
        CHECK(m.fieldText(s.planet, Field::OrbitA) == "149.6 Gm");
        CHECK(m.fieldText(s.planet, Field::OrbitE) == "0");
        CHECK(m.fieldText(s.planet, Field::OrbitOmega) == "0 deg");

        REQUIRE(m.setFieldText(s.planet, Field::OrbitA, "1 AU").ok);
        CHECK(m.body(s.planet)->orbit->a == 149597870700.0);
        REQUIRE(m.setFieldText(s.planet, Field::OrbitA, "0.9 AU").ok);
        REQUIRE(m.setFieldText(s.planet, Field::OrbitE, "0.0167").ok);
        REQUIRE(m.setFieldText(s.planet, Field::OrbitOmega, "102.9").ok);
        REQUIRE(m.setFieldText(s.planet, Field::OrbitMeanAnomaly, "1.5 rad").ok);
        const SceneOrbit& o = *m.body(s.planet)->orbit;
        CHECK(o.a == doctest::Approx(0.9 * 149597870700.0).epsilon(1e-15));
        CHECK(o.e == 0.0167);
        CHECK(o.argPeriapsis_deg == 102.9);
        CHECK(o.meanAnomaly_deg == doctest::Approx(1.5 * 180.0 / 3.14159265358979323846).epsilon(1e-14));

        EditResult r = m.setFieldText(s.planet, Field::OrbitE, "1");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("below 1") != std::string::npos);
        CHECK_FALSE(m.setFieldText(s.planet, Field::OrbitE, "-0.5").ok);
        CHECK_FALSE(m.setFieldText(s.planet, Field::OrbitA, "0").ok);
        CHECK_FALSE(m.setFieldText(s.planet, Field::OrbitA, "5 kg").ok);
        CHECK(m.body(s.planet)->orbit->e == 0.0167);

        // The position and velocity are shown (resolved) but cannot be typed over
        Vec3 r0, v0;
        REQUIRE(m.resolvedState(s.planet, r0, v0));
        CHECK_FALSE(m.fieldText(s.planet, Field::PosX).empty());
        CHECK_FALSE(m.fieldText(s.planet, Field::VelY).empty());
        r = m.setFieldText(s.planet, Field::PosX, "1 AU");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("orbit") != std::string::npos);
        r = m.setFieldText(s.planet, Field::VelX, "1 km/s");
        CHECK_FALSE(r.ok);
    }

    TEST_CASE("bakeState keeps the state and switches to position and velocity")
    {
        EditorModel m;
        m.setScene(lucerna());
        const std::uint64_t domus = idOf(m.scene(), "Domus");
        const std::uint64_t star = idOf(m.scene(), "Lucerna");
        m.setRecentre(Recentre::Never);
        Vec3 rBefore, vBefore;
        REQUIRE(m.resolvedState(domus, rBefore, vBefore));
        REQUIRE(m.inOrbitMode(domus));
        REQUIRE(m.bakeState(domus).ok);
        CHECK_FALSE(m.inOrbitMode(domus));
        Vec3 rAfter, vAfter;
        REQUIRE(m.resolvedState(domus, rAfter, vAfter));
        // The planet keeps its own state; the root of a system that has moons is placed as (given - jacobi) + jacobi, so allow
        // for rounding
        CHECK(norm(rAfter - rBefore) < 1e-12 * norm(rBefore));
        CHECK(norm(vAfter - vBefore) < 1e-12 * norm(vBefore));
        CHECK(sameBits(m.body(domus)->position, rBefore));   // what was stored is exactly the placed state
        CHECK(m.fieldText(domus, Field::OrbitA).empty());
        CHECK(m.bakeState(domus).ok);   // already a state: nothing to do
        CHECK(m.bakeState(star).ok);
        std::string err;
        CHECK(m.validate(err));
    }

    TEST_CASE("deleting a parent freezes its satellites where they were placed")
    {
        EditorModel m;
        m.setScene(lucerna());
        const std::uint64_t star = idOf(m.scene(), "Lucerna");
        const std::uint64_t domus = idOf(m.scene(), "Domus");
        std::uint64_t moon = 0;
        for (const SceneBody& b : m.scene().bodies)
            if (b.orbit && b.orbit->parent == domus) moon = b.id;
        REQUIRE(moon != 0);

        // Auto recentring moves the whole scene when the star goes, so compare what does not depend on the origin
        Vec3 rD0, vD0, rM0, vM0, rF0, vF0;
        const std::uint64_t favilla = idOf(m.scene(), "Favilla");
        REQUIRE(m.resolvedState(domus, rD0, vD0));
        REQUIRE(m.resolvedState(moon, rM0, vM0));
        REQUIRE(m.resolvedState(favilla, rF0, vF0));

        REQUIRE(m.remove(star).ok);
        CHECK(m.body(star) == nullptr);
        CHECK_FALSE(m.inOrbitMode(domus));      // was a child of the star: frozen
        CHECK_FALSE(m.inOrbitMode(favilla));
        CHECK(m.inOrbitMode(moon));             // orbits Domus, which is still there
        std::string err;
        REQUIRE(m.validate(err));

        Vec3 rD1, vD1, rM1, vM1, rF1, vF1;
        REQUIRE(m.resolvedState(domus, rD1, vD1));
        REQUIRE(m.resolvedState(moon, rM1, vM1));
        REQUIRE(m.resolvedState(favilla, rF1, vF1));
        CHECK(norm((rM1 - rD1) - (rM0 - rD0)) < 1e-6 * norm(rM0 - rD0));
        CHECK(norm((rF1 - rD1) - (rF0 - rD0)) < 1e-9 * norm(rF0 - rD0));
        CHECK(norm((vF1 - vD1) - (vF0 - vD0)) < 1e-9 * norm(vF0 - vD0));
    }

    TEST_CASE("deleting a parent with Recentre::Never changes no position at all")
    {
        EditorModel m;
        m.setScene(lucerna());
        m.setRecentre(Recentre::Never);
        const std::uint64_t star = idOf(m.scene(), "Lucerna");
        const std::uint64_t domus = idOf(m.scene(), "Domus");
        const std::uint64_t sh = m.addShuttle().id;
        REQUIRE(m.setOrbitAround(sh, domus, 1.0e6).ok);
        Vec3 r0, v0, s0, sv0;
        REQUIRE(m.resolvedState(domus, r0, v0));
        REQUIRE(m.resolvedState(sh, s0, sv0));
        REQUIRE(m.remove(domus).ok);   // the shuttle's parent
        CHECK_FALSE(m.inOrbitMode(sh));
        Vec3 s1, sv1;
        REQUIRE(m.resolvedState(sh, s1, sv1));
        CHECK(sameBits(s1, s0));
        CHECK(sameBits(sv1, sv0));
        (void)star;
    }

    TEST_CASE("warnings: periapsis inside the parent, overlap, duplicate names")
    {
        EditorModel m;
        const std::uint64_t star = m.addBody(BodyKind::Star).id;
        const std::uint64_t planet = m.addBody(BodyKind::Planet).id;

        // Both at the origin: overlapping at the start
        EditResult a = m.addBody(BodyKind::Moon);
        CHECK(anyContains(a.warnings, "overlaps"));

        // Orbit with the periapsis below the star's surface
        EditResult r = m.setOrbitAround(planet, star, 5.0e8);   // the star's radius is 6.957e8
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "periapsis"));
        r = m.setOrbitAround(planet, star, 1.0e10);
        CHECK_FALSE(anyContains(r.warnings, "periapsis"));
        SceneOrbit o = *m.body(planet)->orbit;
        o.e = 0.9;   // periapsis 1e9 > R, fine; push a smaller
        o.a = 1.0e9;
        r = m.setOrbit(planet, o);
        CHECK(anyContains(r.warnings, "periapsis"));

        // Duplicate names
        r = m.setName(planet, "Star 1");
        CHECK(r.ok);
        CHECK(anyContains(r.warnings, "also used"));
    }

    TEST_CASE("a shuttle inside a body is reported")
    {
        EditorModel m;
        m.addBody(BodyKind::Planet);
        const EditResult r = m.addShuttle(ControllerKind::Coast, {1.0e6, 0.0, 0.0}, {});
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "starts inside"));
        const EditResult far = m.addShuttle(ControllerKind::Coast, {1.0e8, 0.0, 0.0}, {});
        CHECK_FALSE(anyContains(far.warnings, "starts inside"));
    }
}
