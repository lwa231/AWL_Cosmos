// [3d-core] spec: 3D Kepler elements (src/core/kepler3d.h). Skipped by default; the developer runs them with
//   cosmos_tests --no-skip --test-case=*3d-core*kepler*
#include <doctest.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
#include "core/kepler.h"
#include "core/kepler3d.h"
#include "test_3d_core_util.h"

using namespace cosmos;
using namespace core3d_test;

namespace {

const double kMuEarth = 3.986004418e14; // m^3/s^2

Elements3D make(double a, double e, double i, double Omega, double omega, double M)
{
    Elements3D el;
    el.a = a;
    el.e = e;
    el.i = i;
    el.Omega = Omega;
    el.omega = omega;
    el.M = M;
    return el;
}

bool finiteElements(const Elements3D& el)
{
    return std::isfinite(el.a) && std::isfinite(el.e) && std::isfinite(el.p) && std::isfinite(el.q) &&
           std::isfinite(el.h) && std::isfinite(el.i) && std::isfinite(el.Omega) && std::isfinite(el.omega) &&
           std::isfinite(el.nu) && std::isfinite(el.M);
}

void checkRanges(const Elements3D& el)
{
    CHECK(el.i >= 0.0);
    CHECK(el.i <= kPi);
    CHECK(el.Omega >= 0.0);
    CHECK(el.Omega < 2.0 * kPi);
    CHECK(el.omega >= 0.0);
    CHECK(el.omega < 2.0 * kPi);
    CHECK(el.nu > -kPi);
    CHECK(el.nu <= kPi);
    CHECK(el.e >= 0.0);
    CHECK(el.h >= 0.0);
}

// The grid every round trip walks. Equatorial (0, pi), polar and generic inclinations; circular to hyperbolic.
const double kIncl[] = {0.0, 1e-3, 30.0 * kDeg, 90.0 * kDeg, 135.0 * kDeg, 179.0 * kDeg, kPi};
const double kNode[] = {0.0, 1.0, 4.0};
const double kPeri[] = {0.0, 2.0, 5.5};
const double kEcc[] = {0.0, 0.1, 0.6, 0.95, 1.5, 4.0};
const double kMean[] = {-2.5, -0.4, 0.0, 0.3, 2.9};

double semiMajorFor(double e) { return e < 1.0 ? 7.0e6 : -7.0e6; }

} // namespace

TEST_CASE("[3d-core] kepler: textbook state, Curtis example 4.3 (i = 153.2 deg, Omega = 255.3 deg)" * doctest::skip())
{
    // Curtis, Orbital Mechanics for Engineering Students, example 4.3 (km, km/s there; SI here).
    const Vec3 r{-6045.0e3, -3490.0e3, 2500.0e3};
    const Vec3 v{-3.457e3, 6.618e3, 2.533e3};
    const double mu = 398600.0e9;
    const Elements3D el = stateToElements3D(r, v, mu);
    CHECK(el.h == doctest::Approx(58310.0e6).epsilon(2e-4));
    CHECK(el.i / kDeg == doctest::Approx(153.2).epsilon(5e-4));
    CHECK(el.Omega / kDeg == doctest::Approx(255.3).epsilon(5e-4));
    CHECK(el.e == doctest::Approx(0.1712).epsilon(2e-3));
    CHECK(el.omega / kDeg == doctest::Approx(20.07).epsilon(2e-3));
    CHECK(el.nu / kDeg == doctest::Approx(28.45).epsilon(2e-3));
    CHECK(el.a == doctest::Approx(8788.0e3).epsilon(5e-4));
    CHECK(el.p == doctest::Approx(el.h * el.h / mu).epsilon(1e-14));
    CHECK(el.q == doctest::Approx(el.p / (1.0 + el.e)).epsilon(1e-14));
    checkRanges(el);
}

TEST_CASE("[3d-core] kepler: textbook state, Vallado example 2-5 (near-polar, high e)" * doctest::skip())
{
    // Vallado, Fundamentals of Astrodynamics and Applications, example 2-5 (RV2COE).
    const Vec3 r{6524.834e3, 6862.875e3, 6448.296e3};
    const Vec3 v{4.901327e3, 5.533756e3, -1.976341e3};
    const Elements3D el = stateToElements3D(r, v, kMuEarth);
    CHECK(el.p == doctest::Approx(11067.790e3).epsilon(1e-5));
    CHECK(el.a == doctest::Approx(36127.343e3).epsilon(1e-5));
    CHECK(el.e == doctest::Approx(0.832853).epsilon(1e-5));
    CHECK(el.i / kDeg == doctest::Approx(87.870).epsilon(1e-4));
    CHECK(el.Omega / kDeg == doctest::Approx(227.89).epsilon(1e-4));
    CHECK(el.omega / kDeg == doctest::Approx(53.38).epsilon(2e-4));
    CHECK(el.nu / kDeg == doctest::Approx(92.335).epsilon(1e-4));
    checkRanges(el);
}

TEST_CASE("[3d-core] kepler: hand-computed circular polar orbit with Omega = 90 deg" * doctest::skip())
{
    // Circular orbit in the y-z plane. h = r x v = (R V, 0, 0): normal +x, so i = 90 deg and the ascending node
    // z_hat x h points along +y: Omega = 90 deg. e = 0, so omega = 0 and nu is the argument of latitude.
    const double R = 7.0e6;
    const double V = std::sqrt(kMuEarth / R);

    SUBCASE("at the ascending node, moving north")
    {
        const Elements3D el = stateToElements3D({0.0, R, 0.0}, {0.0, 0.0, V}, kMuEarth);
        CHECK(el.e == 0.0);
        CHECK(el.i == doctest::Approx(kPi / 2).epsilon(1e-14));
        CHECK(el.Omega == doctest::Approx(kPi / 2).epsilon(1e-14));
        CHECK(el.omega == 0.0);
        CHECK(std::abs(el.nu) < 1e-14);
        CHECK(el.a == doctest::Approx(R).epsilon(1e-14));
        CHECK(el.h == doctest::Approx(R * V).epsilon(1e-14));
    }
    SUBCASE("over the north pole, a quarter orbit later")
    {
        const Elements3D el = stateToElements3D({0.0, 0.0, R}, {0.0, -V, 0.0}, kMuEarth);
        CHECK(el.e == 0.0);
        CHECK(el.i == doctest::Approx(kPi / 2).epsilon(1e-14));
        CHECK(el.Omega == doctest::Approx(kPi / 2).epsilon(1e-14));
        CHECK(el.omega == 0.0);
        CHECK(el.nu == doctest::Approx(kPi / 2).epsilon(1e-14));
    }
    SUBCASE("and back: elements to state")
    {
        const RelState s = elementsToState3D(make(R, 0.0, kPi / 2, kPi / 2, 0.0, kPi / 2), kMuEarth);
        CHECK(relErr(s.r, {0.0, 0.0, R}) < 1e-14);
        CHECK(relErr(s.v, {0.0, -V, 0.0}) < 1e-14);
    }
}

TEST_CASE("[3d-core] kepler: hand-computed inclined ellipse at periapsis" * doctest::skip())
{
    // Periapsis on the ascending node (omega = 0), Omega = 0, i = 60 deg: periapsis is on +x, and the velocity there is
    // the periapsis speed along the line z = tan(60 deg) y, i.e. (0, cos i, sin i) * v_q.
    const double a = 1.0e7, e = 0.3, i = 60.0 * kDeg;
    const double q = a * (1.0 - e);
    const double vq = std::sqrt(kMuEarth * (1.0 + e) / q);
    const RelState s = elementsToState3D(make(a, e, i, 0.0, 0.0, 0.0), kMuEarth);
    CHECK(relErr(s.r, {q, 0.0, 0.0}) < 1e-14);
    CHECK(relErr(s.v, {0.0, vq * std::cos(i), vq * std::sin(i)}) < 1e-14);

    // The same orbit turned by Omega = 90 deg: periapsis on +y, velocity (-cos i, 0, sin i) * v_q.
    const RelState t = elementsToState3D(make(a, e, i, kPi / 2, 0.0, 0.0), kMuEarth);
    CHECK(relErr(t.r, {0.0, q, 0.0}) < 1e-14);
    CHECK(relErr(t.v, {-vq * std::cos(i), 0.0, vq * std::sin(i)}) < 1e-14);

    // omega = 90 deg instead: periapsis a quarter turn past the node, at (0, q cos i, q sin i), moving along -x.
    const RelState u = elementsToState3D(make(a, e, i, 0.0, kPi / 2, 0.0), kMuEarth);
    CHECK(relErr(u.r, {0.0, q * std::cos(i), q * std::sin(i)}) < 1e-14);
    CHECK(relErr(u.v, {-vq, 0.0, 0.0}) < 1e-14);
}

TEST_CASE("[3d-core] kepler: elements -> state -> elements over i, Omega, omega, e and M" * doctest::skip())
{
    // Non-degenerate cases only (i not 0 or pi, e > 0): every element must come back.
    int n = 0;
    for (double i : kIncl) {
        if (i == 0.0 || i == kPi) continue;
        for (double Om : kNode)
            for (double w : kPeri)
                for (double e : kEcc) {
                    if (e == 0.0) continue;
                    for (double M : kMean) {
                        const Elements3D in = make(semiMajorFor(e), e, i, Om, w, M);
                        const RelState s = elementsToState3D(in, kMuEarth);
                        const Elements3D out = stateToElements3D(s.r, s.v, kMuEarth);
                        CAPTURE(i);
                        CAPTURE(Om);
                        CAPTURE(w);
                        CAPTURE(e);
                        CAPTURE(M);
                        REQUIRE(finiteElements(out));
                        checkRanges(out);
                        CHECK(near(out.a, in.a, 1e-11));
                        CHECK(near(out.e, in.e, 1e-11));
                        CHECK(std::abs(out.i - in.i) < 1e-11);
                        // At i = 1e-3 the node is still defined but is ill-conditioned: allow a looser tolerance.
                        const double tolAngle = i < 1e-2 ? 1e-7 : 1e-10;
                        CHECK(std::abs(angleDiff(out.Omega, in.Omega)) < tolAngle);
                        CHECK(std::abs(angleDiff(out.omega, in.omega)) < tolAngle);
                        CHECK(std::abs(angleDiff(out.M, in.M)) < 1e-9);
                        ++n;
                    }
                }
    }
    CHECK(n == 5 * 3 * 3 * 5 * 5);
}

TEST_CASE("[3d-core] kepler: state -> elements -> state, degenerate cases included (i = 0, i = pi, e = 0)" * doctest::skip())
{
    // Whatever angle convention the degenerate cases use, the state must come back.
    for (double i : kIncl)
        for (double Om : kNode)
            for (double w : kPeri)
                for (double e : kEcc)
                    for (double M : kMean) {
                        const RelState s = elementsToState3D(make(semiMajorFor(e), e, i, Om, w, M), kMuEarth);
                        const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
                        CAPTURE(i);
                        CAPTURE(Om);
                        CAPTURE(w);
                        CAPTURE(e);
                        CAPTURE(M);
                        REQUIRE(finiteElements(el));
                        checkRanges(el);
                        const RelState back = elementsToState3D(el, kMuEarth);
                        CHECK(relErr(back.r, s.r) < 1e-10);
                        CHECK(relErr(back.v, s.v) < 1e-10);
                    }
}

TEST_CASE("[3d-core] kepler: degenerate-case conventions" * doctest::skip())
{
    const double a = 8.0e6;

    SUBCASE("equatorial prograde: i = 0 exactly, Omega = 0, omega from +x")
    {
        const RelState s = elementsToState3D(make(a, 0.2, 0.0, 0.0, 1.2, 0.5), kMuEarth);
        CHECK(s.r.z == 0.0);
        CHECK(s.v.z == 0.0);
        const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
        CHECK(el.i == 0.0);
        CHECK(el.Omega == 0.0);
        CHECK(el.omega == doctest::Approx(1.2).epsilon(1e-12));
        CHECK(el.M == doctest::Approx(0.5).epsilon(1e-12));
    }
    SUBCASE("equatorial retrograde: i = pi exactly, Omega = 0, omega in the direction of motion")
    {
        const RelState s = elementsToState3D(make(a, 0.2, kPi, 0.0, 1.2, 0.5), kMuEarth);
        CHECK(s.r.z == 0.0);
        CHECK(s.v.z == 0.0);
        const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
        CHECK(el.i == kPi);
        CHECK(el.Omega == 0.0);
        CHECK(el.omega == doctest::Approx(1.2).epsilon(1e-12));
        // Periapsis direction is at angle -omega from +x (the R1(-pi) flip): check it from the eccentricity vector.
        const Vec3 h = cross(s.r, s.v);
        const Vec3 ev = cross(s.v, h) / kMuEarth - s.r / norm(s.r);
        CHECK(std::abs(angleDiff(std::atan2(ev.y, ev.x), -1.2)) < 1e-12);
    }
    SUBCASE("equatorial with a node given: Omega and omega fold into one angle")
    {
        const RelState s = elementsToState3D(make(a, 0.2, 0.0, 2.0, 1.0, 0.5), kMuEarth);
        const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
        CHECK(el.Omega == 0.0);
        CHECK(std::abs(angleDiff(el.omega, 3.0)) < 1e-12);
        const RelState t = elementsToState3D(make(a, 0.2, kPi, 2.0, 1.0, 0.5), kMuEarth);
        const Elements3D er = stateToElements3D(t.r, t.v, kMuEarth);
        CHECK(er.Omega == 0.0);
        // As active rotations R3(Omega) R1(pi) R3(w) = R3(Omega - w) R1(pi): periapsis at angle Omega - w = -omega'.
        CHECK(std::abs(angleDiff(er.omega, 1.0 - 2.0)) < 1e-12);
    }
    SUBCASE("circular inclined: e = 0, omega = 0, nu is the argument of latitude")
    {
        const RelState s = elementsToState3D(make(a, 0.0, 0.8, 2.0, 0.0, 1.1), kMuEarth);
        const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
        CHECK(el.e == 0.0);
        CHECK(el.omega == 0.0);
        CHECK(el.i == doctest::Approx(0.8).epsilon(1e-12));
        CHECK(el.Omega == doctest::Approx(2.0).epsilon(1e-12));
        CHECK(el.nu == doctest::Approx(1.1).epsilon(1e-12));
        CHECK(el.M == doctest::Approx(1.1).epsilon(1e-12));
    }
    SUBCASE("circular equatorial: Omega = omega = 0, nu is the true longitude")
    {
        const double R = 7.0e6, V = std::sqrt(kMuEarth / R), th = 2.4;
        const Vec3 r{R * std::cos(th), R * std::sin(th), 0.0};
        const Vec3 v{-V * std::sin(th), V * std::cos(th), 0.0};
        const Elements3D pro = stateToElements3D(r, v, kMuEarth);
        CHECK(pro.i == 0.0);
        CHECK(pro.Omega == 0.0);
        CHECK(pro.omega == 0.0);
        CHECK(pro.nu == doctest::Approx(th).epsilon(1e-12));
        const Elements3D retro = stateToElements3D(r, v * -1.0, kMuEarth);
        CHECK(retro.i == kPi);
        CHECK(retro.Omega == 0.0);
        CHECK(retro.omega == 0.0);
        CHECK(std::abs(angleDiff(retro.nu, -th)) < 1e-12);
    }
    SUBCASE("nearly equatorial stays finite and continuous")
    {
        for (double i : {1e-12, 1e-9, 1e-7, kPi - 1e-9}) {
            CAPTURE(i);
            const RelState s = elementsToState3D(make(a, 0.3, i, 1.0, 2.0, 0.7), kMuEarth);
            const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
            REQUIRE(finiteElements(el));
            const RelState back = elementsToState3D(el, kMuEarth);
            CHECK(relErr(back.r, s.r) < 1e-9);
            CHECK(relErr(back.v, s.v) < 1e-9);
        }
    }
    SUBCASE("parabolic speed (e = 1 to rounding): the plane and the shape are still finite")
    {
        const double R = 7.0e6;
        const double V = std::sqrt(2.0 * kMuEarth / R);
        const Elements3D el = stateToElements3D({R, 0.0, 0.0}, {0.0, 0.0, V}, kMuEarth);
        CHECK(el.i == doctest::Approx(kPi / 2).epsilon(1e-14));
        CHECK(el.Omega == doctest::Approx(0.0)); // node at +x: z_hat x h with h = (0, -R V, 0)
        CHECK(el.e == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(el.p == doctest::Approx(2.0 * R).epsilon(1e-12));
        CHECK(el.q == doctest::Approx(R).epsilon(1e-12));
    }
}

TEST_CASE("[3d-core] kepler: in-plane states reduce to the 2D stateToElements" * doctest::skip())
{
    std::mt19937_64 g(20261105);
    int n = 0;
    for (int k = 0; k < 400; ++k) {
        const double R = 1.0e6 + 1.0e8 * uni(g);
        const double vc = std::sqrt(kMuEarth / R);
        const double th = 2.0 * kPi * uni(g);
        const double speed = vc * (0.2 + 1.3 * uni(g)); // ellipses and hyperbolas
        const double dir = 2.0 * kPi * uni(g);          // any direction, so prograde and retrograde
        const Vec3 r{R * std::cos(th), R * std::sin(th), 0.0};
        const Vec3 v{speed * std::cos(dir), speed * std::sin(dir), 0.0};
        const Elements el2 = stateToElements(r, v, kMuEarth);
        if (std::abs(el2.h) < 1e-3 * R * speed) continue; // keep away from radial
        if (std::abs(el2.e - 1.0) < 1e-6 || el2.e < 1e-6) continue; // near-parabolic, or omega ill-conditioned
        const Elements3D el = stateToElements3D(r, v, kMuEarth);
        CAPTURE(k);
        CHECK(el.i == (el2.retrograde ? kPi : 0.0));
        CHECK(el.Omega == 0.0);
        CHECK(near(el.a, el2.a, 1e-12));
        CHECK(near(el.e, el2.e, 1e-12));
        CHECK(near(el.p, el2.p, 1e-12));
        CHECK(near(el.q, el2.q, 1e-12));
        CHECK(near(el.h, std::abs(el2.h), 1e-12));
        CHECK(std::abs(angleDiff(el.nu, el2.nu)) < 1e-11);
        CHECK(std::abs(angleDiff(el.M, el2.M)) < 1e-10);
        CHECK(std::abs(angleDiff(el.omega, el2.retrograde ? -el2.omega : el2.omega)) < 1e-11);
        ++n;
    }
    CHECK(n > 300);
}

TEST_CASE("[3d-core] kepler: elementsToState3D of toElements3D reduces to the 2D elementsToState, exactly planar" *
          doctest::skip())
{
    for (bool retro : {false, true})
        for (double e : kEcc)
            for (double w : kPeri)
                for (double M : kMean) {
                    Elements el2;
                    el2.a = semiMajorFor(e);
                    el2.e = e;
                    el2.omega = w;
                    el2.M = M;
                    el2.retrograde = retro;
                    const RelState s2 = elementsToState(el2, kMuEarth);
                    const Elements3D el3 = toElements3D(el2);
                    CHECK(el3.i == (retro ? kPi : 0.0));
                    CHECK(el3.Omega == 0.0);
                    CHECK(el3.omega >= 0.0);
                    CHECK(el3.omega < 2.0 * kPi);
                    const RelState s3 = elementsToState3D(el3, kMuEarth);
                    CAPTURE(retro);
                    CAPTURE(e);
                    CAPTURE(w);
                    CAPTURE(M);
                    CHECK(s3.r.z == 0.0);
                    CHECK(s3.v.z == 0.0);
                    CHECK(relErr(s3.r, s2.r) < 1e-12);
                    CHECK(relErr(s3.v, s2.v) < 1e-12);
                }

    // toElements3D copies the shape and the anomalies, and drops the sign of h.
    const Elements el2 = stateToElements({7.0e6, 1.0e6, 0.0}, {-1000.0, -7000.0, 0.0}, kMuEarth);
    REQUIRE(el2.retrograde);
    const Elements3D el3 = toElements3D(el2);
    CHECK(el3.a == el2.a);
    CHECK(el3.e == el2.e);
    CHECK(el3.p == el2.p);
    CHECK(el3.q == el2.q);
    CHECK(el3.nu == el2.nu);
    CHECK(el3.M == el2.M);
    CHECK(el3.h == -el2.h);
}

TEST_CASE("[3d-core] kepler: orbit normal, energy and |h| of the 3D state" * doctest::skip())
{
    for (double i : kIncl)
        for (double Om : kNode)
            for (double e : {0.0, 0.5, 2.0}) {
                const double a = semiMajorFor(e);
                const Elements3D el = make(a, e, i, Om, 1.3, 0.4);
                const RelState s = elementsToState3D(el, kMuEarth);
                const Vec3 h = cross(s.r, s.v);
                const Vec3 n = orbitNormal(el);
                CAPTURE(i);
                CAPTURE(Om);
                CAPTURE(e);
                CHECK(std::abs(norm(n) - 1.0) < 1e-15);
                CHECK(relErr(h / norm(h), n) < 1e-12);
                // h^2 = mu a (1 - e^2) and vis-viva
                CHECK(near(norm2(h), kMuEarth * a * (1.0 - e * e), 1e-11));
                CHECK(near(0.5 * norm2(s.v) - kMuEarth / norm(s.r), -kMuEarth / (2.0 * a), 1e-11));
            }
    CHECK(orbitNormal(make(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)).x == 0.0);
    CHECK(orbitNormal(make(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)).y == 0.0);
    CHECK(orbitNormal(make(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)).z == 1.0);
    const Vec3 down = orbitNormal(make(1.0, 0.0, kPi, 1.0, 0.0, 0.0));
    CHECK(down.x == 0.0);
    CHECK(down.y == 0.0);
    CHECK(down.z == -1.0);
}

TEST_CASE("[3d-core] kepler: rotating the state about z only shifts Omega (or omega when equatorial)" * doctest::skip())
{
    const RelState s = elementsToState3D(make(9.0e6, 0.4, 50.0 * kDeg, 0.3, 1.0, -1.0), kMuEarth);
    const Elements3D el = stateToElements3D(s.r, s.v, kMuEarth);
    for (double t : {0.5, 2.0, -1.5}) {
        const Elements3D rot = stateToElements3D(rotZ(s.r, t), rotZ(s.v, t), kMuEarth);
        CAPTURE(t);
        CHECK(std::abs(angleDiff(rot.Omega, el.Omega + t)) < 1e-12);
        CHECK(std::abs(angleDiff(rot.omega, el.omega)) < 1e-12);
        CHECK(rot.i == doctest::Approx(el.i).epsilon(1e-13));
        CHECK(std::abs(angleDiff(rot.nu, el.nu)) < 1e-12);
    }
}
