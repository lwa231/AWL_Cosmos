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

// Barycentric system. The Sun is placed from momentum balance; the planets get the two-body state for mu = mu_sun + mu_planet.
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
    const RelState rs = elementsToState(es, MU_SUN + MU_SATURN);

    BodyDesc sun, jup, sat;
    sun.mu = MU_SUN;
    jup.mu = MU_JUPITER;
    jup.position = rj.r;
    jup.velocity = rj.v;
    sat.mu = MU_SATURN;
    sat.position = rs.r;
    sat.velocity = rs.v;

    const double mt = MU_SUN + MU_JUPITER + MU_SATURN;
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

// The tabulated a are mean elements. Used as osculating a they make Saturn run 0.7 % too fast (Jupiter's pull changes the
// mean motion relative to Kepler's law for the osculating a), which doubles 2 n_J - 5 n_S and halves the Great Inequality
// period (measured 430 yr instead of 884). So the osculating a of both planets are adjusted, three fixed-point rounds of
// 1500-yr runs, until the integrated mean longitudes advance at the tabulated rates L-dot (measured results 5.2042 and
// 9.5821 AU).
void calibratedSemiMajorAxes(double& aJ, double& aS)
{
    static double cachedJ = 0.0, cachedS = 0.0;
    if (cachedJ == 0.0) {
        cachedJ = JUPITER.a;
        cachedS = SATURN.a;
        const double nJ = JUPITER.Ldot * DEG / 100.0, nS = SATURN.Ldot * DEG / 100.0; // rad per year
        for (int iter = 0; iter < 3; ++iter) {
            Bodies b = makeSystem(cachedJ, cachedS);
            const Series s = runSystem(b, 1e-9, 1500);
            cachedJ *= std::pow(slope(s.lambdaJ) / nJ, 2.0 / 3.0);
            cachedS *= std::pow(slope(s.lambdaS) / nS, 2.0 / 3.0);
        }
    }
    aJ = cachedJ;
    aS = cachedS;
}
}

TEST_SUITE("ias15 adaptive: Sun-Jupiter-Saturn")
{
    TEST_CASE("Great Inequality: Jupiter's osculating a oscillates with a period of 880-940 yr; a tighter epsilon agrees to 0.1%")
    {
        double aJ, aS;
        calibratedSemiMajorAxes(aJ, aS);

        Bodies b1 = makeSystem(aJ, aS);
        const Series s1 = runSystem(b1, 1e-9, 6000);
        Bodies b2 = makeSystem(aJ, aS);
        const Series s2 = runSystem(b2, 1e-11, 6000);

        double amp1 = 0.0, amp2 = 0.0;
        const double p1 = dominantPeriod(s1.aJupiter, 500.0, 1500.0, &amp1);
        const double p2 = dominantPeriod(s2.aJupiter, 500.0, 1500.0, &amp2);

        // the resonance angle 2 lambda_J - 5 lambda_S circulates once per 884 yr. Jupiter's a oscillates a few percent slower
        // (measured 937.7 yr): the perihelion longitudes, which precess, enter the full resonant argument (5 L_S - 2 L_J - 3 varpi).
        std::vector<double> phi(s1.lambdaJ.size());
        for (std::size_t k = 0; k < phi.size(); ++k) phi[k] = 2.0 * s1.lambdaJ[k] - 5.0 * s1.lambdaS[k];
        const double pPhi = TWO_PI / std::abs(slope(phi));

        // Measured: calibrated osculating a = 5.2042 AU (Jupiter), 9.5821 AU (Saturn); period 937.66 yr at both epsilon
        // 1e-9 (29410 steps) and 1e-11 (53516 steps), amplitude 2.2e-4 AU; resonance angle period 882.1 yr;
        // energy error 3.6e-15 and 4.1e-15 over 6000 yr.
        CHECK(p1 > 880.0);
        CHECK(p1 < 940.0);
        CHECK(std::abs(p1 - p2) < 0.001 * p1);
        CHECK(std::abs(amp1 - amp2) < 0.01 * amp1);
        CHECK(pPhi > 870.0); // the resonance angle 2 L_J - 5 L_S turns once per 884 yr, as 2 n_J - 5 n_S = -0.0071 rad/yr says
        CHECK(pPhi < 895.0);
        CHECK(amp1 > 1e-4); // a real oscillation, not noise: a few 1e-4 AU
        CHECK(s1.maxEnergyError < 1e-13);
        CHECK(s2.maxEnergyError < 1e-13);
    }
}
