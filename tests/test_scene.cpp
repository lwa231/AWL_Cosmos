#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "core/diagnostics.h"
#include "core/kepler.h"
#include "core/radii.h"
#include "core/scene.h"
#include "test_scene_util.h"

using namespace cosmos;
using scene_test::Lucerna;

namespace {

const double kPi = 3.141592653589793238462643383279502884;
const double kDeg = kPi / 180.0;

double wrapPi(double a)
{
    double t = std::fmod(a + kPi, 2.0 * kPi);
    if (t <= 0.0) t += 2.0 * kPi;
    return t - kPi;
}

// Scene indices of a body and everything that orbits it, directly or not
void subsystem(const Scene& sc, std::size_t root, std::vector<std::size_t>& out)
{
    out.push_back(root);
    for (std::size_t i = 0; i < sc.bodies.size(); ++i)
        if (sc.bodies[i].orbit && sc.bodies[i].orbit->parent == sc.bodies[root].id) subsystem(sc, i, out);
}

double subsystemMu(const Scene& sc, std::size_t root)
{
    std::vector<std::size_t> members;
    subsystem(sc, root, members);
    double mu = 0.0;
    for (std::size_t i : members) mu += sc.bodies[i].mu;
    return mu;
}

// Plan 5.9: mu of the parent plus the subsystems of the inner siblings plus the body's own subsystem
double jacobiMu(const Scene& sc, std::size_t k)
{
    const SceneBody& b = sc.bodies[k];
    double mu = sc.bodies[static_cast<std::size_t>(findSceneBody(sc, b.orbit->parent))].mu + subsystemMu(sc, k);
    for (std::size_t s = 0; s < sc.bodies.size(); ++s) {
        const SceneBody& o = sc.bodies[s];
        if (s != k && o.orbit && o.orbit->parent == b.orbit->parent && o.orbit->a < b.orbit->a) mu += subsystemMu(sc, s);
    }
    return mu;
}

double jacobiPeriod(const Scene& sc, std::size_t k)
{
    const double a = sc.bodies[k].orbit->a;
    return 2.0 * kPi * std::sqrt(a * a * a / jacobiMu(sc, k));
}

struct Bary {
    double mu = 0.0;
    Vec3 r, v;
};

Bary barycentre(const Lucerna& L, const std::vector<std::size_t>& sceneIdx)
{
    Bary g;
    const BodyState& s = L.res.bodies.state;
    for (std::size_t i : sceneIdx) {
        const std::size_t k = *L.res.bodies.find(L.res.bodyIds[i]);
        const double mu = L.res.bodies.mu[k];
        g.mu += mu;
        g.r += mu * Vec3{s.x[k], s.y[k], s.z[k]};
        g.v += mu * Vec3{s.vx[k], s.vy[k], s.vz[k]};
    }
    g.r = g.r / g.mu;
    g.v = g.v / g.mu;
    return g;
}

// Elements of body k's subsystem about the barycentre of its parent plus its inner siblings' subsystems, read back from
// the resolved states
Elements recovered(const Lucerna& L, std::size_t k)
{
    const Scene& sc = L.scene;
    const SceneBody& b = sc.bodies[k];
    const std::size_t p = static_cast<std::size_t>(findSceneBody(sc, b.orbit->parent));

    std::vector<std::size_t> inner{p};
    for (std::size_t s = 0; s < sc.bodies.size(); ++s) {
        const SceneBody& o = sc.bodies[s];
        if (s != k && o.orbit && o.orbit->parent == b.orbit->parent && o.orbit->a < b.orbit->a) subsystem(sc, s, inner);
    }
    std::vector<std::size_t> own;
    subsystem(sc, k, own);

    const Bary in = barycentre(L, inner);
    const Bary ow = barycentre(L, own);
    return stateToElements(ow.r - in.r, ow.v - in.v, in.mu + ow.mu);
}

} // namespace

TEST_SUITE("scene")
{
    TEST_CASE("Lucerna resolves: 14 bodies, scene IDs kept, barycentre at rest at the origin")
    {
        Lucerna L;
        CHECK(L.res.bodies.size() == 14);
        CHECK(L.res.bodies.consistent());
        CHECK(L.res.idsPreserved);
        CHECK(L.res.warnings.empty());
        for (std::size_t i = 0; i < L.scene.bodies.size(); ++i) CHECK(L.res.bodyIds[i] == L.scene.bodies[i].id);

        const Conserved c = conserved(L.res.bodies);
        double muTotal = 0.0, speedScale = 0.0;
        Vec3 r;
        for (std::size_t i = 0; i < L.res.bodies.size(); ++i) {
            const BodyState& s = L.res.bodies.state;
            muTotal += L.res.bodies.mu[i];
            r += L.res.bodies.mu[i] * Vec3{s.x[i], s.y[i], s.z[i]};
            speedScale += L.res.bodies.mu[i] * std::hypot(s.vx[i], s.vy[i]);
        }
        CHECK(norm(r) / muTotal < 1e-6);                  // metres, on a system 1e11 m across (measured 3e-10)
        CHECK(norm(c.momentumG) / speedScale < 1e-14);    // relative to the sum of |p| (measured 3e-17)
        for (std::size_t i = 0; i < L.res.bodies.size(); ++i) CHECK(L.res.bodies.state.z[i] == 0.0);
    }

    TEST_CASE("Lucerna fields come through: kind, locks, atmosphere, colour, fictional density")
    {
        Lucerna L;
        const Bodies& B = L.res.bodies;
        CHECK(B.kind[L.storeIndex("Lucerna")] == BodyKind::Star);
        CHECK(B.kind[L.storeIndex("Maxima")] == BodyKind::GasGiant);
        CHECK(B.kind[L.storeIndex("Hiems")] == BodyKind::DwarfPlanet);
        CHECK(B.kind[L.storeIndex("Gelu")] == BodyKind::Moon);
        for (std::size_t i = 0; i < B.size(); ++i) {
            CHECK(B.kindLocked[i] == 1);
            CHECK(B.fictionalDensity[i] == 1);
        }
        const std::size_t domus = L.storeIndex("Domus");
        REQUIRE(B.atmosphere[domus].has_value());
        CHECK(B.atmosphere[domus]->rho0 == 1.22);
        CHECK(B.atmosphere[domus]->scaleHeight == 7000.0);
        CHECK_FALSE(B.atmosphere[L.storeIndex("Custos")].has_value());
        CHECK(B.colour[domus][0] == 0.28f);
        CHECK(B.colour[domus][2] == 0.78f);

        // Gear-marked values (plan 13): Gelu a = 4.5e7, Ignis a = 2.5e7, Ignis spin 12.49 h
        CHECK(L.body("Gelu").orbit->a == 4.5e7);
        CHECK(L.body("Ignis").orbit->a == 2.5e7);
        CHECK(L.body("Ignis").spinPeriod_h == 12.49);
    }

    TEST_CASE("spin periods become signed rad/s; Velum and Exul spin retrograde")
    {
        Lucerna L;
        const Bodies& B = L.res.bodies;
        const double domus = B.spinRate[L.storeIndex("Domus")];
        CHECK(domus == doctest::Approx(2.0 * kPi / (5.8 * 3600.0)).epsilon(1e-15));
        CHECK(domus > 0.0);
        CHECK(B.spinRate[L.storeIndex("Velum")] == doctest::Approx(-2.0 * kPi / (1560.0 * 3600.0)).epsilon(1e-15));
        CHECK(B.spinRate[L.storeIndex("Exul")] == doctest::Approx(-2.0 * kPi / (77.47 * 3600.0)).epsilon(1e-15));
        CHECK(B.spinRate[L.storeIndex("Exul")] < 0.0);
        CHECK(B.spinRate[L.storeIndex("Lucerna")] > 0.0);

        // Locked moons spin once per orbit (plan 13 table); Ignis's adjusted 12.49 h matches its Jacobi period
        const double ignisPeriodH = jacobiPeriod(L.scene, L.sceneIndex("Ignis")) / 3600.0;
        CHECK(std::fabs(ignisPeriodH - 12.49) < 0.005);
    }

    TEST_CASE("plan 13 derived numbers: surface gravity")
    {
        Lucerna L;
        struct Row { const char* name; double g; double tol; };
        // Table value and half a unit of its last printed digit
        const Row rows[] = {
            {"Lucerna", 21.7, 0.05}, {"Favilla", 3.11, 0.005}, {"Velum", 11.79, 0.005}, {"Domus", 9.80, 0.005},
            {"Custos", 1.05, 0.005}, {"Gelu", 0.30, 0.005},    {"Rubor", 3.20, 0.005},  {"Exul", 0.050, 0.0005},
            {"Maxima", 7.45, 0.005}, {"Ignis", 2.20, 0.005},   {"Unda", 2.00, 0.005},   {"Aura", 4.00, 0.005},
            {"Hiems", 0.95, 0.005},  {"Sodalis", 0.50, 0.005},
        };
        for (const Row& r : rows) {
            const std::size_t i = L.storeIndex(r.name);
            const double g = L.res.bodies.mu[i] / (L.res.bodies.radius[i] * L.res.bodies.radius[i]);
            INFO(r.name << ": g = " << g);
            CHECK(std::fabs(g - r.g) <= r.tol);
        }
    }

    TEST_CASE("plan 13 derived numbers: Domus escape speed, low orbit, synchronous orbit, year")
    {
        Lucerna L;
        const SceneBody& domus = L.body("Domus");
        CHECK(std::sqrt(2.0 * domus.mu / domus.radius) == doctest::Approx(3486.0).epsilon(1e-4));   // 3485.9

        const double r = domus.radius + 1.0e5;   // 100 km low orbit
        CHECK(circularSpeed(domus.mu, r) == doctest::Approx(2287.0).epsilon(2e-4));                  // 2287.3
        CHECK(2.0 * kPi * std::sqrt(r * r * r / domus.mu) / 60.0 == doctest::Approx(33.0).epsilon(2e-3));   // 32.96 min

        const double T = std::fabs(domus.spinPeriod_h) * 3600.0;
        const double rSync = std::cbrt(domus.mu * T * T / (4.0 * kPi * kPi));
        CHECK((rSync - domus.radius) / 1e3 == doctest::Approx(2845.0).epsilon(1e-4));                // 2844.97 km

        const double year = jacobiPeriod(L.scene, L.sceneIndex("Domus"));
        CHECK(year == doctest::Approx(9.81e6).epsilon(1e-3));                                        // 9.8124e6 s
        CHECK(year / 86400.0 == doctest::Approx(113.6).epsilon(1e-3));                               // Earth days

        const double rubor = jacobiPeriod(L.scene, L.sceneIndex("Rubor")) / year;
        const double maxima = jacobiPeriod(L.scene, L.sceneIndex("Maxima")) / year;
        const double hiems = jacobiPeriod(L.scene, L.sceneIndex("Hiems")) / year;
        CHECK(rubor == doctest::Approx(1.81).epsilon(2e-3));
        CHECK(maxima == doctest::Approx(8.00).epsilon(2e-3));
        CHECK(hiems == doctest::Approx(25.9).epsilon(2e-3));
    }

    TEST_CASE("plan 13 derived numbers: atmosphere tops (rho = 1e-6 kg/m^3)")
    {
        Lucerna L;
        struct Row { const char* name; double km; };
        const Row rows[] = {{"Domus", 98.0}, {"Velum", 138.0}, {"Rubor", 117.0}, {"Maxima", 772.0}, {"Aura", 89.0}};
        for (const Row& r : rows) {
            const Atmosphere& a = *L.body(r.name).atmosphere;
            const double topKm = a.scaleHeight * std::log(a.rho0 / 1e-6) / 1e3;
            INFO(r.name << ": top = " << topKm << " km");
            CHECK(std::fabs(topKm - r.km) <= 0.5);   // the table is rounded to the km
        }
    }

    TEST_CASE("plan 13 derived numbers: Hill radii at a and at periapsis")
    {
        Lucerna L;
        const double muStar = L.body("Lucerna").mu;
        struct Row { const char* name; double atA; double atPeri; };
        const Row rows[] = {
            {"Domus", 1.452e8, 1.430e8}, {"Rubor", 9.94e7, 9.34e7}, {"Maxima", 2.513e9, 2.425e9}, {"Hiems", 3.02e8, 2.66e8},
        };
        for (const Row& r : rows) {
            const SceneBody& b = L.body(r.name);
            const double atA = hillRadius(b.orbit->a, 0.0, b.mu, muStar);
            const double atPeri = hillRadius(b.orbit->a, b.orbit->e, b.mu, muStar);
            INFO(r.name << ": " << atA << " / " << atPeri);
            CHECK(atA == doctest::Approx(r.atA).epsilon(2e-3));
            CHECK(atPeri == doctest::Approx(r.atPeri).epsilon(2e-3));
        }
    }

    TEST_CASE("plan 13 derived numbers: moon periods with Jacobi mu")
    {
        Lucerna L;
        struct Row { const char* name; double hours; double tol; };
        const Row rows[] = {
            {"Custos", 18.72, 0.005}, {"Gelu", 269.7, 0.05}, {"Exul", 77.47, 0.005}, {"Ignis", 12.49, 0.005},
            {"Unda", 41.84, 0.005},   {"Aura", 324.9, 0.05}, {"Sodalis", 30.84, 0.005},
        };
        for (const Row& r : rows) {
            const double h = jacobiPeriod(L.scene, L.sceneIndex(r.name)) / 3600.0;
            INFO(r.name << ": " << h << " h");
            CHECK(std::fabs(h - r.hours) <= r.tol);
        }
    }

    TEST_CASE("Jacobi placement is exact: every body's orbit is read back from the resolved states")
    {
        Lucerna L;
        for (std::size_t k = 0; k < L.scene.bodies.size(); ++k) {
            const SceneBody& b = L.scene.bodies[k];
            if (!b.orbit) continue;
            const Elements el = recovered(L, k);
            INFO(b.name);
            CHECK(el.a == doctest::Approx(b.orbit->a).epsilon(1e-10));
            CHECK(el.e == doctest::Approx(b.orbit->e).epsilon(1e-10));   // absolute for e = 0 as well
            CHECK(el.retrograde == b.orbit->retrograde);
            // omega + M is the angle that survives e -> 0; omega alone only means something for e > 1e-6 (plan 14)
            const double sum = (b.orbit->argPeriapsis_deg + b.orbit->meanAnomaly_deg) * kDeg;
            CHECK(std::fabs(wrapPi(el.omega + el.M - sum)) < 1e-9);
            if (b.orbit->e > 1e-6) CHECK(std::fabs(wrapPi(el.omega - b.orbit->argPeriapsis_deg * kDeg)) < 1e-9);
        }
    }

    TEST_CASE("Exul moves clockwise about Rubor, Custos counter-clockwise about Domus")
    {
        Lucerna L;
        const BodyState& s = L.res.bodies.state;
        auto lz = [&](const char* moon, const char* planet) {
            const std::size_t m = L.storeIndex(moon), p = L.storeIndex(planet);
            return (s.x[m] - s.x[p]) * (s.vy[m] - s.vy[p]) - (s.y[m] - s.y[p]) * (s.vx[m] - s.vx[p]);
        };
        CHECK(lz("Exul", "Rubor") < 0.0);
        CHECK(lz("Custos", "Domus") > 0.0);
    }

    TEST_CASE("empty scene resolves to an empty store")
    {
        Scene sc;
        ResolvedScene out;
        std::string err;
        REQUIRE(resolveScene(sc, out, err));
        CHECK(out.bodies.size() == 0);
        CHECK(out.shuttles.empty());
    }

    TEST_CASE("explicit states are taken verbatim; orbit bodies around them are placed relative to the root body")
    {
        Scene sc;
        SceneBody star;
        star.id = 1; star.name = "S"; star.mu = 1.0e14; star.radius = 1e3;
        star.position = {1.0e9, -2.0e9, 0.0};
        star.velocity = {3.0, 4.0, 0.0};
        SceneBody loner;   // a second, unrelated root
        loner.id = 2; loner.name = "L"; loner.mu = 1.0; loner.radius = 1.0;
        loner.position = {-5.0e9, 0.0, 0.0};
        loner.velocity = {0.0, -7.0, 0.0};
        SceneBody planet;
        planet.id = 3; planet.name = "P"; planet.mu = 1.0e9; planet.radius = 1e2;
        planet.orbit = SceneOrbit{1, 1.0e8, 0.0, 0.0, 0.0, false};
        sc.bodies = {star, loner, planet};

        ResolveOptions never;
        never.recentre = Recentre::Never;
        ResolvedScene out;
        std::string err;
        REQUIRE_MESSAGE(resolveScene(sc, out, err, never), err);
        const BodyState& s = out.bodies.state;
        // The root bodies sit exactly where the scene put them
        CHECK(s.x[0] == 1.0e9);
        CHECK(s.y[0] == -2.0e9);
        CHECK(s.vx[0] == 3.0);
        CHECK(s.vy[0] == 4.0);
        CHECK(s.x[1] == -5.0e9);
        CHECK(s.vy[1] == -7.0);
        // The planet is on a circular orbit of radius a about the star body... (mu = mu_star + mu_planet; e = 0, M = 0)
        const double dx = s.x[2] - s.x[0], dy = s.y[2] - s.y[0];
        const double dvx = s.vx[2] - s.vx[0], dvy = s.vy[2] - s.vy[0];
        // ...up to the star's tiny offset from the pair's barycentre (mu ratio 1e-5)
        CHECK(std::hypot(dx, dy) == doctest::Approx(1.0e8).epsilon(1e-11));
        CHECK(std::hypot(dvx, dvy) == doctest::Approx(std::sqrt((1.0e14 + 1.0e9) / 1.0e8)).epsilon(1e-11));

        // Auto with an orbit present moves everything to the barycentre at rest...
        ResolvedScene moved;
        REQUIRE(resolveScene(sc, moved, err));
        Vec3 r, v;
        double mu = 0.0;
        for (std::size_t i = 0; i < moved.bodies.size(); ++i) {
            mu += moved.bodies.mu[i];
            r += moved.bodies.mu[i] * Vec3{moved.bodies.state.x[i], moved.bodies.state.y[i], moved.bodies.state.z[i]};
            v += moved.bodies.mu[i] * Vec3{moved.bodies.state.vx[i], moved.bodies.state.vy[i], moved.bodies.state.vz[i]};
        }
        CHECK(norm(r) / mu < 1e-3);
        CHECK(norm(v) / mu < 1e-9);
        // ...while the relative geometry is unchanged
        CHECK(moved.bodies.state.x[1] - moved.bodies.state.x[0] == doctest::Approx(-6.0e9).epsilon(1e-12));

        // Auto with no orbit anywhere leaves the scene alone
        sc.bodies.pop_back();
        ResolvedScene verbatim;
        REQUIRE(resolveScene(sc, verbatim, err));
        CHECK(verbatim.bodies.state.x[0] == 1.0e9);
        CHECK(verbatim.bodies.state.vy[1] == -7.0);
    }

    TEST_CASE("IDs: gaps are bridged, out-of-reach IDs fall back to the next free store ID with a warning")
    {
        Scene sc;
        for (std::uint64_t id : {7u, 3u, 5000u}) {
            SceneBody b;
            b.id = id; b.name = "B" + std::to_string(id); b.mu = 1.0; b.radius = 1.0;
            b.position = {static_cast<double>(id), 0.0, 0.0};
            sc.bodies.push_back(b);
        }
        ResolvedScene out;
        std::string err;
        REQUIRE_MESSAGE(resolveScene(sc, out, err), err);
        CHECK(out.idsPreserved);
        CHECK(out.bodyIds == std::vector<BodyId>{7, 3, 5000});
        // Store order is ascending ID, not file order
        CHECK(out.bodies.ids == std::vector<BodyId>{3, 7, 5000});
        CHECK(out.bodies.state.x[0] == 3.0);
        CHECK(out.bodies.consistent());

        sc.bodies[0].id = 4000000000ull;
        REQUIRE(resolveScene(sc, out, err));
        CHECK_FALSE(out.idsPreserved);
        CHECK(out.warnings.size() == 1);
        CHECK(out.bodyIds[1] == 3);            // reachable ones are still kept
        CHECK(out.bodyIds[2] == 5000);
        CHECK(out.bodyIds[0] == 5001);         // the unreachable one goes after the highest kept ID
        CHECK(out.bodies.consistent());
    }

    TEST_CASE("shuttles: orbit about a body is converted against that body's resolved state")
    {
        Lucerna L;
        Scene sc = L.scene;
        SceneShuttle low;
        low.id = 1001; low.name = "Low"; low.controller = "Prograde";
        low.orbit = SceneOrbit{4, 7.2e5, 0.0, 0.0, 0.0, false};   // 100 km above Domus
        SceneShuttle retro = low;
        retro.id = 1002; retro.name = "Retro"; retro.orbit->retrograde = true; retro.orbit->meanAnomaly_deg = 90.0;
        SceneShuttle fixed;
        fixed.id = 1003; fixed.name = "Fixed"; fixed.position = {1.0, 2.0, 0.0}; fixed.velocity = {3.0, 4.0, 0.0};
        sc.shuttles = {low, retro, fixed};

        ResolvedScene out;
        std::string err;
        REQUIRE_MESSAGE(resolveScene(sc, out, err), err);
        REQUIRE(out.shuttles.size() == 3);
        const BodyState& s = out.bodies.state;
        const std::size_t d = *out.bodies.find(out.bodyIds[L.sceneIndex("Domus")]);
        const double muD = out.bodies.mu[d];

        for (int k = 0; k < 2; ++k) {
            const ResolvedShuttle& sh = out.shuttles[static_cast<std::size_t>(k)];
            const Vec3 rel = sh.position - Vec3{s.x[d], s.y[d], s.z[d]};
            const Vec3 vrel = sh.velocity - Vec3{s.vx[d], s.vy[d], s.vz[d]};
            CHECK(norm(rel) == doctest::Approx(7.2e5).epsilon(1e-12));
            CHECK(norm(vrel) == doctest::Approx(std::sqrt(muD / 7.2e5)).epsilon(1e-12));
            CHECK(std::fabs(dot(rel, vrel)) / (norm(rel) * norm(vrel)) < 1e-12);    // circular: velocity is perpendicular
            const double lz = rel.x * vrel.y - rel.y * vrel.x;
            CHECK((k == 0 ? lz > 0.0 : lz < 0.0));
        }
        // Retrograde means clockwise, so M = 90 deg is a quarter turn clockwise from +x: on -y of Domus
        CHECK(out.shuttles[1].position.y - s.y[d] == doctest::Approx(-7.2e5).epsilon(1e-12));

        // An explicit state lives in the scene's frame, which resolveScene moves like the bodies: it keeps its offset from Lucerna
        const std::size_t star = L.storeIndex("Lucerna");
        CHECK(out.shuttles[2].position.x - s.x[star] == doctest::Approx(1.0).epsilon(1e-5));
        CHECK(out.shuttles[2].position.y - s.y[star] == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(out.shuttles[2].velocity.y - s.vy[star] == doctest::Approx(4.0).epsilon(1e-11));
        CHECK(out.shuttles[0].desc.controller == "Prograde");
        CHECK(out.shuttles[0].desc.dryMass == 2000.0);   // 7.1 defaults carried along
    }

    TEST_CASE("invalid scenes are refused with a message naming the culprit")
    {
        auto bodyAt = [](std::uint64_t id, const char* name) {
            SceneBody b;
            b.id = id; b.name = name; b.mu = 1.0e12; b.radius = 1e3;
            return b;
        };
        auto orbiting = [&](std::uint64_t id, const char* name, std::uint64_t parent) {
            SceneBody b = bodyAt(id, name);
            b.orbit = SceneOrbit{parent, 1.0e7, 0.1, 0.0, 0.0, false};
            return b;
        };
        struct Case { Scene sc; const char* expect; };
        std::vector<Case> cases;

        { Scene s; s.bodies = {bodyAt(1, "A"), bodyAt(1, "B")}; cases.push_back({s, "duplicate id"}); }
        { Scene s; s.bodies = {bodyAt(0, "A")}; cases.push_back({s, "id must be nonzero"}); }
        { Scene s; SceneBody b = bodyAt(1, "A"); b.mu = 0.0; s.bodies = {b}; cases.push_back({s, "mu must be > 0"}); }
        { Scene s; SceneBody b = bodyAt(1, "A"); b.radius = -1.0; s.bodies = {b}; cases.push_back({s, "radius"}); }
        { Scene s; s.bodies = {bodyAt(1, "A"), orbiting(2, "B", 9)}; cases.push_back({s, "parent id 9 does not exist"}); }
        { Scene s; s.bodies = {bodyAt(1, "A"), orbiting(2, "B", 2)}; cases.push_back({s, "orbits itself"}); }
        { Scene s; s.bodies = {orbiting(1, "A", 2), orbiting(2, "B", 1)}; cases.push_back({s, "cycle"}); }
        { Scene s; SceneBody b = orbiting(2, "B", 1); b.orbit->e = 1.0; s.bodies = {bodyAt(1, "A"), b}; cases.push_back({s, "eccentricity"}); }
        { Scene s; SceneBody b = orbiting(2, "B", 1); b.orbit->a = -5.0; s.bodies = {bodyAt(1, "A"), b}; cases.push_back({s, "semi-major axis"}); }
        { Scene s; SceneBody b = bodyAt(1, "A"); b.position.x = std::nan(""); s.bodies = {b}; cases.push_back({s, "not finite"}); }
        { Scene s; s.settings.controllerTick_s = 0.0; cases.push_back({s, "controllerTick_s"}); }
        { Scene s; SceneShuttle sh; sh.id = 1; sh.position = {}; s.bodies = {bodyAt(1, "A")}; s.shuttles = {sh}; cases.push_back({s, "already used"}); }
        { Scene s; SceneShuttle sh; sh.id = 50; sh.orbit = SceneOrbit{77, 1e6, 0, 0, 0, false}; s.bodies = {bodyAt(1, "A")}; s.shuttles = {sh}; cases.push_back({s, "not a body"}); }

        for (const Case& c : cases) {
            ResolvedScene out;
            std::string err;
            INFO("expecting: " << c.expect);
            CHECK_FALSE(resolveScene(c.sc, out, err));
            CHECK(err.find(c.expect) != std::string::npos);
            CHECK(out.bodies.size() == 0);   // nothing half-built is handed back
        }
    }
}
