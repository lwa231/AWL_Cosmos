#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "core/bodies.h"
#include "core/constants.h"
#include "core/gravity.h"
#include "core/ias15.h"
#include "core/kepler.h"

using namespace cosmos;

// Plan section 14: Sun-Jupiter-Saturn, coplanar, real mu, approximate J2000 elements. The Great Inequality is the
// 5:2 near-resonance, 2 n_J - 5 n_S = -0.0071 rad/yr, period 2 pi / 0.0071 = 884 yr. It shows up as a slow oscillation
// (initial conditions: J2000 table elements placed in Jacobi coordinates, with osculating a corrected once, see below)
// of Jupiter's osculating semi-major axis (on top of the 12 yr, 20 yr and 63 yr terms).

namespace {
const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const double DEG = PI / 180.0;
const double YEAR = 365.25 * 86400.0;
const double MU_SUN = 1.32712440018e20;
const double MU_JUPITER = 1.26686534e17;
const double MU_SATURN = 3.7931187e16;

// JPL "Keplerian elements for approximate positions of the major planets", table 1 (1800-2050), J2000
struct Planet {
    double a, e, varpi, L; // AU, -, deg (longitude of perihelion), deg (mean longitude)
    double Ldot;           // deg per Julian century
};
const Planet JUPITER = {5.20288700, 0.04838624, 14.72847983, 34.39644051, 3034.9056606};
const Planet SATURN = {9.53667594, 0.05386179, 92.59887831, 49.95424423, 1222.1138488};

// Osculating semi-major axes of the start state (AU), used instead of the table's mean a.
// Why they differ: the table gives MEAN elements, a fit to positions over 1800-2050 with the short-period terms of the
// planets' mutual perturbations averaged out. A state vector needs OSCULATING elements, which differ from the mean ones
// by exactly those terms. The mean motion of the integrated orbit is not the Kepler value of the osculating a either. The
// 2:5 resonance makes this matter: 2 n_J - 5 n_S is 0.0071 rad/yr, a difference of rates of 0.53 and 0.21 rad/yr, so a 4e-4
// relative error in n_J (which the raw table a gives, even in Jacobi coordinates) moves the Great Inequality period from
// 937 yr to 1045 yr. There is no closed form short of a perturbation theory, so a was adjusted ONCE, offline: five
// fixed-point rounds a *= (n_integrated / n_table)^(2/3) on 1500-yr runs (epsilon 1e-9), starting from the table's a, until
// the slope of the unwrapped osculating mean longitude equalled the tabulated rate L-dot to 1e-8. The first test case
// below re-measures that (to 1e-4); it fails if the initial conditions or the integrator change the dynamics.
// Result: Jupiter +2.8e-4 and Saturn -2.0e-4 relative to the table.
const double A_JUPITER_OSC = 5.204304129;
const double A_SATURN_OSC = 9.535462773;

// Barycentric system from the table's elements, placed in Jacobi coordinates (the hierarchy Sun, Jupiter, Saturn):
//   * Jupiter's elements are relative to the Sun, two-body mu = mu_sun + mu_jupiter;
//   * Saturn's elements are relative to the barycentre of Sun + Jupiter, two-body mu = mu_sun + mu_jupiter + mu_saturn.
// The tabulated elements are mean elements: the mean motion they imply is the one a planet has when it orbits the whole mass
// inside its orbit. Heliocentric two-body placement of Saturn is wrong at the percent level, because the Sun itself
// swings around the Sun-Jupiter barycentre at 12.5 m/s (1.3e-3 of Saturn's 9.7 km/s) and the heliocentric velocity of
// Saturn inherits that swing, which changes the osculating a and n of Saturn by ~0.3 % and 2 n_J - 5 n_S, a difference of
// two nearly equal rates (-0.0071 rad/yr against 0.53 rad/yr, i.e. 1.3 %), by tens of percent. Jacobi coordinates remove that
// error; what remains (4e-4 in n_J, 2.5e-4 in n_S) is the mean-versus-osculating difference described at A_JUPITER_OSC.
Bodies makeSystem(double aJupiterAU, double aSaturnAU)
{
    Elements ej;
    ej.a = aJupiterAU * constants::AU;
    ej.e = JUPITER.e;
    ej.omega = JUPITER.varpi * DEG;
    ej.M = (JUPITER.L - JUPITER.varpi) * DEG;
    Elements es;
    es.a = aSaturnAU * constants::AU;
    es.e = SATURN.e;
    es.omega = SATURN.varpi * DEG;
    es.M = (SATURN.L - SATURN.varpi) * DEG;
    const RelState rj = elementsToState(ej, MU_SUN + MU_JUPITER);
    const RelState rs = elementsToState(es, MU_SUN + MU_JUPITER + MU_SATURN);

    // positions and velocities with the Sun at the origin
    const double m01 = MU_SUN + MU_JUPITER;
    const Vec3 c01 = (MU_JUPITER / m01) * rj.r; // barycentre of Sun + Jupiter
    const Vec3 u01 = (MU_JUPITER / m01) * rj.v;
    BodyDesc sun, jup, sat;
    sun.mu = MU_SUN;
    jup.mu = MU_JUPITER;
    jup.position = rj.r;
    jup.velocity = rj.v;
    sat.mu = MU_SATURN;
    sat.position = c01 + rs.r;
    sat.velocity = u01 + rs.v;

    // move to the total barycentre
    const double mt = m01 + MU_SATURN;
    const Vec3 cm = (MU_JUPITER * jup.position + MU_SATURN * sat.position) / mt;
    const Vec3 cv = (MU_JUPITER * jup.velocity + MU_SATURN * sat.velocity) / mt;
    sun.position = Vec3{} - cm;
    sun.velocity = Vec3{} - cv;
    jup.position -= cm;
    jup.velocity -= cv;
    sat.position -= cm;
    sat.velocity -= cv;

    Bodies b;
    b.add(sun);
    b.add(jup);
    b.add(sat);
    return b;
}


double totalEnergy(const Bodies& b)
{
    double e = 0.0;
    const BodyState& s = b.state;
    for (std::size_t i = 0; i < 3; ++i) e += 0.5 * b.mu[i] * (s.vx[i] * s.vx[i] + s.vy[i] * s.vy[i]);
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = i + 1; j < 3; ++j) e -= b.mu[i] * b.mu[j] / std::hypot(s.x[i] - s.x[j], s.y[i] - s.y[j]);
    return e;
}

// Heliocentric osculating elements of planet index k (1 = Jupiter, 2 = Saturn)
Elements heliocentric(const Bodies& b, std::size_t k)
{
    const BodyState& s = b.state;
    const Vec3 r{s.x[k] - s.x[0], s.y[k] - s.y[0], 0.0};
    const Vec3 v{s.vx[k] - s.vx[0], s.vy[k] - s.vy[0], 0.0};
    return stateToElements(r, v, b.mu[0] + b.mu[k]);
}

// Samples once per year
struct Series {
    std::vector<double> aJupiter; // osculating semi-major axis, AU
    std::vector<double> lambdaJ;  // unwrapped osculating mean longitude (varpi + M), rad
    std::vector<double> lambdaS;
    double maxEnergyError = 0.0;
    long steps = 0;
    long rejections = 0;
};

void pushUnwrapped(std::vector<double>& v, double lam)
{
    if (!v.empty()) lam -= TWO_PI * std::round((lam - v.back()) / TWO_PI);
    v.push_back(lam);
}

// Integrate for `years`, landing exactly on every whole year (this also exercises tMax clipping)
Series runSystem(Bodies& b, double epsilon, int years)
{
    IAS15Params p;
    p.epsilon = epsilon;
    IAS15 ias(p);
    const ForceFn f = [&b](const BodyState& s, Accel& a) { computeAccelerations(s, b.mu, a); };
    const double E0 = totalEnergy(b);
    Series out;
    double t = 0.0;
    for (int k = 0; k <= years; ++k) {
        if (k > 0) {
            const double tEnd = k * YEAR;
            while (t < tEnd) ias.step(b.state, f, t, tEnd);
            out.maxEnergyError = std::max(out.maxEnergyError, std::abs((totalEnergy(b) - E0) / E0));
        }
        const Elements ej = heliocentric(b, 1), es = heliocentric(b, 2);
        out.aJupiter.push_back(ej.a / constants::AU);
        pushUnwrapped(out.lambdaJ, ej.omega + ej.M);
        pushUnwrapped(out.lambdaS, es.omega + es.M);
    }
    out.steps = ias.totalSteps();
    out.rejections = ias.totalRejections();
    return out;
}

// Least-squares slope of y against the sample index
double slope(const std::vector<double>& y)
{
    const double n = static_cast<double>(y.size());
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (std::size_t k = 0; k < y.size(); ++k) {
        sx += k;
        sy += y[k];
        sxx += static_cast<double>(k) * k;
        sxy += k * y[k];
    }
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

// Solve the 4x4 system A x = r by Gaussian elimination with partial pivoting
void solve4(double A[4][4], double r[4], double x[4])
{
    for (int c = 0; c < 4; ++c) {
        int piv = c;
        for (int i = c + 1; i < 4; ++i)
            if (std::abs(A[i][c]) > std::abs(A[piv][c])) piv = i;
        for (int j = 0; j < 4; ++j) std::swap(A[c][j], A[piv][j]);
        std::swap(r[c], r[piv]);
        for (int i = c + 1; i < 4; ++i) {
            const double f = A[i][c] / A[c][c];
            for (int j = c; j < 4; ++j) A[i][j] -= f * A[c][j];
            r[i] -= f * r[c];
        }
    }
    for (int i = 3; i >= 0; --i) {
        double s = r[i];
        for (int j = i + 1; j < 4; ++j) s -= A[i][j] * x[j];
        x[i] = s / A[i][i];
    }
}

// Period (in samples) of the strongest sinusoid with a period between pMin and pMax. For each trial period P the series is
// fitted by least squares with  c0 + c1 t + c2 cos(2 pi t / P) + c3 sin(2 pi t / P)  (a straight line takes care of slow
// drifts); the P with the largest amplitude wins, scanned in steps of 0.5 and refined by a parabola through its neighbours.
double dominantPeriod(const std::vector<double>& y, double pMin, double pMax, double* amplitude = nullptr)
{
    const std::size_t n = y.size();
    const double kMid = 0.5 * static_cast<double>(n - 1);
    std::vector<double> amp;
    for (double P = pMin; P <= pMax; P += 0.5) {
        double A[4][4] = {}, r[4] = {}, x[4] = {};
        for (std::size_t k = 0; k < n; ++k) {
            const double u = (static_cast<double>(k) - kMid) / kMid;
            const double ph = TWO_PI * static_cast<double>(k) / P;
            const double col[4] = {1.0, u, std::cos(ph), std::sin(ph)};
            for (int i = 0; i < 4; ++i) {
                r[i] += col[i] * y[k];
                for (int j = 0; j < 4; ++j) A[i][j] += col[i] * col[j];
            }
        }
        solve4(A, r, x);
        amp.push_back(std::sqrt(x[2] * x[2] + x[3] * x[3]));
    }
    std::size_t best = 1;
    for (std::size_t i = 1; i + 1 < amp.size(); ++i)
        if (amp[i] > amp[best]) best = i;
    const double y0 = amp[best - 1], y1 = amp[best], y2 = amp[best + 1];
    if (amplitude) *amplitude = y1;
    return pMin + 0.5 * (static_cast<double>(best) + 0.5 * (y0 - y2) / (y0 - 2.0 * y1 + y2));
}

}

TEST_SUITE("ias15 adaptive: Sun-Jupiter-Saturn")
{
    TEST_CASE("the osculating a of the start state reproduce the tabulated mean motions (this is what the constants are for)")
    {
        // 1500 yr is long enough (125 Jupiter orbits, 50 Saturn orbits) for the slope of the unwrapped osculating mean
        // longitude to be the mean motion to ~1e-5. Measured: n_J / n_table = 0.9999976 and n_S / n_table = 1.0000149.
        // With the raw table a (in Jacobi coordinates) the same run gives 1.00042 and 0.99975, and that error alone moves
        // the period of the Great Inequality from 937 yr to 1045 yr.
        Bodies b = makeSystem(A_JUPITER_OSC, A_SATURN_OSC);
        const Series s = runSystem(b, 1e-9, 1500);
        const double nJ = JUPITER.Ldot * DEG / 100.0, nS = SATURN.Ldot * DEG / 100.0; // rad per year
        CHECK(std::abs(slope(s.lambdaJ) / nJ - 1.0) < 1e-4);
        CHECK(std::abs(slope(s.lambdaS) / nS - 1.0) < 1e-4);
        // and the correction is small: the osculating a differ from the table's mean a by a few 1e-4 (relative)
        CHECK(std::abs(A_JUPITER_OSC / JUPITER.a - 1.0) < 5e-4);
        CHECK(std::abs(A_SATURN_OSC / SATURN.a - 1.0) < 5e-4);
    }

    TEST_CASE("Great Inequality: Jupiter's osculating a oscillates with a period of 880-1000 yr; a tighter epsilon agrees to 0.1%")
    {
        Bodies b1 = makeSystem(A_JUPITER_OSC, A_SATURN_OSC);
        const Series s1 = runSystem(b1, 1e-9, 6000);
        Bodies b2 = makeSystem(A_JUPITER_OSC, A_SATURN_OSC);
        const Series s2 = runSystem(b2, 1e-11, 6000);

        double amp1 = 0.0, amp2 = 0.0;
        const double p1 = dominantPeriod(s1.aJupiter, 500.0, 1500.0, &amp1);
        const double p2 = dominantPeriod(s2.aJupiter, 500.0, 1500.0, &amp2);

        // The resonance angle 2 lambda_J - 5 lambda_S circulates once per 2 pi / |2 n_J - 5 n_S| = 883 yr (from the table's
        // rates). Jupiter's a oscillates a few percent slower: the perihelion longitudes, which precess, enter the full
        // resonant argument, which is of third order in the eccentricities (2 L_J - 5 L_S + 3 varpi combinations).
        std::vector<double> phi(s1.lambdaJ.size());
        for (std::size_t k = 0; k < phi.size(); ++k) phi[k] = 2.0 * s1.lambdaJ[k] - 5.0 * s1.lambdaS[k];
        const double pPhi = TWO_PI / std::abs(slope(phi));

        // Measured: period 937.4 yr at both epsilon 1e-9 (29446 steps) and 1e-11 (53564 steps), amplitude 2.29e-4 AU,
        // resonance angle period 881.0 yr, energy error 2.7e-15 and 1.7e-15 over 6000 yr. The windows are the physical
        // expectation (about 880-940 yr, which is 884 yr plus the precession of the perihelia) with 6 % of margin around
        // the measurement, so a change of summation order or of libm cannot flip them; the real regression guards are
        // the epsilon agreement (0.1 %) and the energy.
        CHECK(p1 > 880.0);
        CHECK(p1 < 1000.0);
        CHECK(p1 > pPhi); // slower than the pure resonance angle
        CHECK(std::abs(p1 - p2) < 0.001 * p1);
        CHECK(std::abs(amp1 - amp2) < 0.01 * amp1);
        CHECK(pPhi > 860.0); // 2 n_J - 5 n_S = -0.0071 rad/yr says 883 yr
        CHECK(pPhi < 905.0);
        CHECK(amp1 > 1e-4); // a real oscillation, not noise: a few 1e-4 AU
        CHECK(s1.maxEnergyError < 1e-13);
        CHECK(s2.maxEnergyError < 1e-13);
    }
}
