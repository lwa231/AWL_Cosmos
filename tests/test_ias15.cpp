#include <doctest.h>
#include <algorithm>
#include <cmath>
#include "core/ias15.h"

using namespace cosmos;

namespace {
// Gauss-Radau (left, 8 points) nodes and weights on [0, 1], computed at 40 digits.
const double NODES[8] = {
    0.0,
    0.05626256053692214646565219,
    0.1802406917368923649875799,
    0.3526247171131696373739078,
    0.5471536263305553830014486,
    0.7342101772154105315232106,
    0.8853209468390957680903598,
    0.9775206135612875018911745};

const double WEIGHTS[8] = {
    0.015625,
    0.09267907740148963927036449,
    0.1520653103233925644878716,
    0.1882587726945592782860646,
    0.1957860837262467965412498,
    0.173507397817250640114338,
    0.1248239506649324816289346,
    0.05725440737212859967117686};

// a(tau) = F0 + b[0] tau + b[1] tau^2 + ... + b[6] tau^7
double polyAt(double F0, const double b[7], double tau)
{
    double r = 0.0;
    for (int k = 6; k >= 0; --k) r = (r + b[k]) * tau;
    return F0 + r;
}

double maxAbsDiff(const double* p, const double* q, int n)
{
    double m = 0.0;
    for (int i = 0; i < n; ++i) m = std::max(m, std::abs(p[i] - q[i]));
    return m;
}

// Fit the polynomial with coefficients b through the nodes and return what bFromG(gFromF(F)) recovers.
double roundTripError(double F0, const double bTrue[7])
{
    double F[8], g[8] = {0}, b[7];
    for (int n = 0; n < 8; ++n) F[n] = polyAt(F0, bTrue, ias15::h[n]);
    ias15::gFromF(F, g);
    ias15::bFromG(g, b);
    return maxAbsDiff(b, bTrue, 7);
}
}

TEST_SUITE("ias15 nodes")
{
    TEST_CASE("first node is exactly zero, the rest increase inside (0, 1)")
    {
        CHECK(ias15::h[0] == 0.0);
        for (int n = 1; n < 8; ++n) {
            CHECK(ias15::h[n] > ias15::h[n - 1]);
            CHECK(ias15::h[n] < 1.0);
        }
    }

    TEST_CASE("nodes match the table to 1e-15")
    {
        for (int n = 0; n < 8; ++n) CHECK(std::abs(ias15::h[n] - NODES[n]) < 1e-15);
    }

    TEST_CASE("Radau quadrature on these nodes is exact for every monomial up to degree 14")
    {
        for (int k = 0; k <= 14; ++k) {
            double sum = 0.0;
            for (int i = 0; i < 8; ++i) sum += WEIGHTS[i] * std::pow(ias15::h[i], k);
            CHECK(std::abs(sum - 1.0 / (k + 1)) < 1e-14);
        }
    }
}

TEST_SUITE("ias15 g and b")
{
    TEST_CASE("constant acceleration: every divided difference is zero")
    {
        double F[8], g[8] = {0};
        for (int n = 0; n < 8; ++n) F[n] = 3.5;
        ias15::gFromF(F, g);
        for (int k = 1; k < 8; ++k) CHECK(std::abs(g[k]) < 1e-12);
    }

    TEST_CASE("linear acceleration a = F0 + s*tau: g1 = s, the rest zero")
    {
        const double F0 = 2.0, s = -1.5;
        double F[8], g[8] = {0};
        for (int n = 0; n < 8; ++n) F[n] = F0 + s * ias15::h[n];
        ias15::gFromF(F, g);
        CHECK(std::abs(g[1] - s) < 1e-13);
        for (int k = 2; k < 8; ++k) CHECK(std::abs(g[k]) < 1e-12);
    }

    TEST_CASE("quadratic a = F0 + tau*(tau - h1): g2 = 1, and the others vanish")
    {
        double F[8], g[8] = {0};
        for (int n = 0; n < 8; ++n) F[n] = 1.0 + ias15::h[n] * (ias15::h[n] - ias15::h[1]);
        ias15::gFromF(F, g);
        CHECK(std::abs(g[1]) < 1e-13);   // g1 = (F1 - F0)/h1 = 0 for this polynomial
        CHECK(std::abs(g[2] - 1.0) < 1e-12);
        for (int k = 3; k < 8; ++k) CHECK(std::abs(g[k]) < 1e-12);
    }

    TEST_CASE("bFromG by hand: only g1 gives b0 = g1, the rest 0")
    {
        const double g[8] = {0, 2.5, 0, 0, 0, 0, 0, 0};
        double b[7];
        ias15::bFromG(g, b);
        CHECK(b[0] == doctest::Approx(2.5).epsilon(1e-15).scale(0));
        for (int k = 1; k < 7; ++k) CHECK(std::abs(b[k]) < 1e-15);
    }

    TEST_CASE("bFromG by hand: only g2 gives tau*(tau - h1) = tau^2 - h1*tau")
    {
        const double g[8] = {0, 0, 4.0, 0, 0, 0, 0, 0};
        double b[7];
        ias15::bFromG(g, b);
        CHECK(std::abs(b[0] - (-ias15::h[1] * 4.0)) < 1e-15);
        CHECK(std::abs(b[1] - 4.0) < 1e-15);
        for (int k = 2; k < 7; ++k) CHECK(std::abs(b[k]) < 1e-15);
    }

    TEST_CASE("bFromG by hand: only g3 gives tau*(tau - h1)*(tau - h2)")
    {
        const double g[8] = {0, 0, 0, 1.0, 0, 0, 0, 0};
        double b[7];
        ias15::bFromG(g, b);
        CHECK(std::abs(b[0] - ias15::h[1] * ias15::h[2]) < 1e-15);
        CHECK(std::abs(b[1] - (-(ias15::h[1] + ias15::h[2]))) < 1e-15);
        CHECK(std::abs(b[2] - 1.0) < 1e-15);
        for (int k = 3; k < 7; ++k) CHECK(std::abs(b[k]) < 1e-15);
    }

    TEST_CASE("round trip: samples of a known polynomial give back its coefficients")
    {
        const double sets[3][7] = {
            { 0.3, -0.7,  0.5,  0.9, -0.2,  0.4, -0.8},
            {-1.0,  1.0, -1.0,  1.0, -1.0,  1.0, -1.0},
            { 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0},
        };
        for (const auto& b : sets) CHECK(roundTripError(0.7, b) < 1e-10);
    }

    TEST_CASE("round trip: the fitted polynomial passes through the samples")
    {
        const double bTrue[7] = {0.3, -0.7, 0.5, 0.9, -0.2, 0.4, -0.8};
        double F[8], g[8] = {0}, b[7];
        for (int n = 0; n < 8; ++n) F[n] = polyAt(0.7, bTrue, ias15::h[n]);
        ias15::gFromF(F, g);
        ias15::bFromG(g, b);
        for (int n = 0; n < 8; ++n) CHECK(std::abs(polyAt(0.7, b, ias15::h[n]) - F[n]) < 1e-12);
    }
}

TEST_SUITE("ias15 polynomial integration")
{
    // positionAt(x0, v0, a0, b, hh, dt) and velocityAt(v0, a0, b, hh, dt): one component, evaluated at tau = hh.

    TEST_CASE("constant acceleration a0: x = x0 + v0 dt h + a0 (dt h)^2 / 2, v = v0 + a0 dt h")
    {
        const double b[7] = {0, 0, 0, 0, 0, 0, 0};
        const double x0 = 2.0, v0 = 3.0, a0 = 1.0, dt = 0.5, hh = 0.4;
        CHECK(ias15::positionAt(x0, v0, a0, b, hh, dt)
              == doctest::Approx(x0 + v0 * dt * hh + 0.5 * a0 * dt * dt * hh * hh).epsilon(1e-15).scale(0));
        CHECK(ias15::velocityAt(v0, a0, b, hh, dt)
              == doctest::Approx(v0 + a0 * dt * hh).epsilon(1e-15).scale(0));
    }

    TEST_CASE("each b_k alone: x = dt^2 h^(k+3) / ((k+2)(k+3)), v = dt h^(k+2) / (k+2)")
    {
        const double dt = 0.7, hh = 0.9;
        for (int k = 0; k < 7; ++k) {
            double b[7] = {0, 0, 0, 0, 0, 0, 0};
            b[k] = 1.0;
            const double xExpected = dt * dt * std::pow(hh, k + 3) / ((k + 2) * (k + 3));
            const double vExpected = dt * std::pow(hh, k + 2) / (k + 2);
            CHECK(ias15::positionAt(0.0, 0.0, 0.0, b, hh, dt) == doctest::Approx(xExpected).epsilon(1e-14).scale(0));
            CHECK(ias15::velocityAt(0.0, 0.0, b, hh, dt) == doctest::Approx(vExpected).epsilon(1e-14).scale(0));
        }
    }

    TEST_CASE("velocity is the time derivative of position")
    {
        const double b[7] = {0.3, -0.7, 0.5, 0.9, -0.2, 0.4, -0.8};
        const double x0 = 1.0, v0 = -2.0, a0 = 0.7, dt = 0.6, hh = 0.55, eps = 1e-5;
        const double fd = (ias15::positionAt(x0, v0, a0, b, hh + eps, dt)
                         - ias15::positionAt(x0, v0, a0, b, hh - eps, dt)) / (2.0 * eps * dt);
        CHECK(std::abs(fd - ias15::velocityAt(v0, a0, b, hh, dt)) < 1e-8);
    }

    TEST_CASE("acceleration is the time derivative of velocity")
    {
        const double b[7] = {0.3, -0.7, 0.5, 0.9, -0.2, 0.4, -0.8};
        const double v0 = -2.0, a0 = 0.7, dt = 0.6, hh = 0.55, eps = 1e-5;
        const double fd = (ias15::velocityAt(v0, a0, b, hh + eps, dt)
                         - ias15::velocityAt(v0, a0, b, hh - eps, dt)) / (2.0 * eps * dt);
        CHECK(std::abs(fd - polyAt(a0, b, hh)) < 1e-8);
    }
}
