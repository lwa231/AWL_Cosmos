#include <doctest.h>
#include <cmath>
#include <limits>
#include "core/dopri5.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Dormand-Prince 5(4) building blocks (plan 6.4): coefficients, order of the step, error estimate, step control.

namespace {
struct PointMass {
    double mu;
    Vec3 operator()(double, const Vec3& r) const
    {
        const double d2 = norm2(r);
        return r * (-mu / (d2 * std::sqrt(d2)));
    }
};

// Fixed-step Kepler orbit (mu = 1, a = 1) for one period, N equal steps. Returns the final position error.
double keplerError(double e, int N, double* errEstimate = nullptr)
{
    const PointMass f{1.0};
    const double P = 2.0 * PI;
    const double h = P / N;
    Dopri5State y;
    y.r = {1.0 - e, 0.0, 0.0};
    y.v = {0.0, std::sqrt((1.0 + e) / (1.0 - e)), 0.0};
    const Dopri5State y0 = y;
    Vec3 a = f(0.0, y.r);
    double lastErr = 0.0;
    for (int k = 0; k < N; ++k) {
        const Dopri5Result res = dopri5Step(f, k * h, h, y, a);
        y = res.y1;
        a = res.a1;
        lastErr = norm(res.err.r);
    }
    if (errEstimate) *errEstimate = lastErr;
    return norm(y.r - y0.r); // after exactly one period the exact solution is back at the start
}
} // namespace

TEST_SUITE("dopri5")
{
    TEST_CASE("coefficients: row sums equal the nodes, weights satisfy the order conditions")
    {
        using namespace dopri5;
        for (int i = 0; i < 7; ++i) {
            double s = 0.0;
            for (int j = 0; j < 6; ++j) s += a[i][j];
            CHECK(s == doctest::Approx(c[i]).epsilon(1e-15).scale(1.0));
        }
        // FSAL: the fifth-order weights are the last row of a
        for (int j = 0; j < 6; ++j) CHECK(b5[j] == a[6][j]);
        CHECK(b5[6] == 0.0);
        // Quadrature conditions sum b_i c_i^k = 1/(k+1): fifth order up to k = 4, the embedded fourth order up to k = 3
        for (int k = 0; k <= 4; ++k) {
            double s5 = 0.0, s4 = 0.0;
            for (int i = 0; i < 7; ++i) {
                s5 += b5[i] * std::pow(c[i], k);
                s4 += b4[i] * std::pow(c[i], k);
            }
            CAPTURE(k);
            CHECK(s5 == doctest::Approx(1.0 / (k + 1)).epsilon(1e-14));
            if (k <= 3) CHECK(s4 == doctest::Approx(1.0 / (k + 1)).epsilon(1e-14));
            else CHECK(std::fabs(s4 - 1.0 / (k + 1)) > 1e-4); // the embedded method really is only fourth order
        }
        // Tree conditions of order 3 and 4 (both methods): sum b_i a_ij c_j = 1/6, sum b_i c_i a_ij c_j = 1/8,
        // sum b_i a_ij c_j^2 = 1/12, sum b_i a_ij a_jk c_k = 1/24
        for (const double* b : {b5, b4}) {
            double t3 = 0.0, t4a = 0.0, t4b = 0.0, t4c = 0.0;
            for (int i = 0; i < 7; ++i)
                for (int j = 0; j < i; ++j) {
                    t3 += b[i] * a[i][j] * c[j];
                    t4a += b[i] * c[i] * a[i][j] * c[j];
                    t4b += b[i] * a[i][j] * c[j] * c[j];
                    for (int k = 0; k < j; ++k) t4c += b[i] * a[i][j] * a[j][k] * c[k];
                }
            CHECK(t3 == doctest::Approx(1.0 / 6.0).epsilon(1e-14));
            CHECK(t4a == doctest::Approx(1.0 / 8.0).epsilon(1e-14));
            CHECK(t4b == doctest::Approx(1.0 / 12.0).epsilon(1e-14));
            CHECK(t4c == doctest::Approx(1.0 / 24.0).epsilon(1e-14));
        }
        // Error weights are b5 - b4 and sum to zero
        double se = 0.0;
        for (int i = 0; i < 7; ++i) {
            CHECK(e[i] == doctest::Approx(b5[i] - b4[i]).epsilon(1e-14).scale(1e-3));
            se += e[i];
        }
        CHECK(std::fabs(se) < 1e-16);
    }

    TEST_CASE("order 5: halving the step cuts the Kepler error by about 2^5 = 32 (e = 0.3 and e = 0.5)")
    {
        // Global error after one period. Measured ratios 31.9 .. 37.3 once the steps are small enough (the constant of the
        // leading error term makes them a little larger than 32); the pre-asymptotic range N < 100 is not used.
        struct Case { double e; int first; };
        for (const Case& c : {Case{0.3, 100}, Case{0.5, 200}}) {
            double prev = keplerError(c.e, c.first);
            for (int N = 2 * c.first; N <= 1600; N *= 2) {
                const double cur = keplerError(c.e, N);
                CAPTURE(c.e);
                CAPTURE(N);
                CHECK(prev / cur > 30.0);
                CHECK(prev / cur < 40.0);
                prev = cur;
            }
        }
        CHECK(keplerError(0.3, 1600) < 1e-12);
        // A fourth-order method would give 16 and a sixth-order one 64
    }

    TEST_CASE("FSAL: the returned end acceleration is the field at the new position and time")
    {
        const PointMass f{1.0};
        Dopri5State y{{1.0, 0.0, 0.0}, {0.0, 1.1, 0.0}};
        const Dopri5Result res = dopri5Step(f, 0.0, 0.1, y, f(0.0, y.r));
        const Vec3 a1 = f(0.1, res.y1.r);
        CHECK(res.a1.x == a1.x);
        CHECK(res.a1.y == a1.y);
        CHECK(res.a1.z == a1.z);
    }

    TEST_CASE("the error estimate scales as h^5 (local error of the embedded method) and bounds the error of the result")
    {
        const PointMass f{1.0};
        Dopri5State y{{1.0, 0.0, 0.0}, {0.0, 1.2, 0.0}};
        const Vec3 a0 = f(0.0, y.r);
        const double e1 = norm(dopri5Step(f, 0.0, 0.1, y, a0).err.r);
        const double e2 = norm(dopri5Step(f, 0.0, 0.05, y, a0).err.r);
        const double e3 = norm(dopri5Step(f, 0.0, 0.025, y, a0).err.r);
        CHECK(e1 / e2 > 28.0); // measured 31.9 and 32.0
        CHECK(e1 / e2 < 36.0);
        CHECK(e2 / e3 > 28.0);
        CHECK(e2 / e3 < 36.0);
        // The result is the fifth-order solution, so its true error (against the same step split into 256 substeps) is much
        // smaller than the estimate, which belongs to the embedded fourth-order solution (measured 0.04 of it at h = 0.05)
        for (double h : {0.1, 0.05, 0.025}) {
            Dopri5State ref = y;
            Vec3 a = a0;
            for (int k = 0; k < 256; ++k) {
                const Dopri5Result r = dopri5Step(f, k * h / 256, h / 256, ref, a);
                ref = r.y1;
                a = r.a1;
            }
            const Dopri5Result one = dopri5Step(f, 0.0, h, y, a0);
            CAPTURE(h);
            CHECK(norm(one.y1.r - ref.r) < 0.1 * norm(one.err.r));
            CHECK(norm(one.y1.v - ref.v) < 0.3 * norm(one.err.v)); // velocity: 0.2 of it at h = 0.1
        }
    }

    TEST_CASE("exact on polynomial motion: constant acceleration is integrated without error")
    {
        auto f = [](double, const Vec3&) { return Vec3{0.0, -9.81, 0.0}; };
        Dopri5State y{{0.0, 0.0, 0.0}, {30.0, 40.0, 0.0}};
        const Dopri5Result res = dopri5Step(f, 0.0, 7.0, y, f(0.0, y.r));
        CHECK(res.y1.r.x == doctest::Approx(210.0).epsilon(1e-14));
        CHECK(res.y1.r.y == doctest::Approx(40.0 * 7.0 - 0.5 * 9.81 * 49.0).epsilon(1e-14));
        CHECK(res.y1.v.y == doctest::Approx(40.0 - 9.81 * 7.0).epsilon(1e-14));
        CHECK(norm(res.err.r) < 1e-12);
        CHECK(norm(res.err.v) < 1e-12);
    }

    TEST_CASE("time-dependent acceleration is sampled at the stage times (harmonic drive)")
    {
        // r'' = cos(t) with r(0) = 0, v(0) = 0 has r = 1 - cos(t), v = sin(t)
        auto f = [](double t, const Vec3&) { return Vec3{std::cos(t), 0.0, 0.0}; };
        Dopri5State y{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        Vec3 a = f(0.0, y.r);
        const int N = 100;
        const double h = 2.0 / N;
        for (int k = 0; k < N; ++k) {
            const Dopri5Result r = dopri5Step(f, k * h, h, y, a);
            y = r.y1;
            a = r.a1;
        }
        CHECK(y.r.x == doctest::Approx(1.0 - std::cos(2.0)).epsilon(1e-10));
        CHECK(y.v.x == doctest::Approx(std::sin(2.0)).epsilon(1e-10));
    }

    TEST_CASE("step-size factor min(5, max(0.2, 0.9 err^(-1/5)))")
    {
        CHECK(dopri5StepFactor(1.0) == doctest::Approx(0.9).epsilon(1e-15));
        CHECK(dopri5StepFactor(0.5) == doctest::Approx(0.9 * std::pow(0.5, -0.2)).epsilon(1e-15));
        CHECK(dopri5StepFactor(32.0) == doctest::Approx(0.9 * 0.5).epsilon(1e-15));
        CHECK(dopri5StepFactor(1e-10) == 5.0);  // 0.9 * 100 capped
        CHECK(dopri5StepFactor(0.0) == 5.0);
        CHECK(dopri5StepFactor(1e5) == 0.2);    // 0.9 * 0.1 floored
        CHECK(dopri5StepFactor(std::numeric_limits<double>::infinity()) == 0.2);
        CHECK(dopri5StepFactor(std::numeric_limits<double>::quiet_NaN()) == 0.2);
    }

    TEST_CASE("error norm is the RMS of err / scale over six components; non-finite errors give infinity")
    {
        Dopri5State err{{3.0, 0.0, 0.0}, {0.0, 4.0, 0.0}};
        // positions scaled by 3, velocities by 2: components 1, 0, 0, 0, 2, 0 -> sqrt(5 / 6)
        CHECK(dopri5ErrorNorm(err, 3.0, 2.0) == doctest::Approx(std::sqrt(5.0 / 6.0)).epsilon(1e-15));
        err.r.z = std::numeric_limits<double>::quiet_NaN();
        CHECK(std::isinf(dopri5ErrorNorm(err, 3.0, 2.0)));
        err.r.z = std::numeric_limits<double>::infinity();
        CHECK(std::isinf(dopri5ErrorNorm(err, 3.0, 2.0)));
    }

    TEST_CASE("tolerance scales: position by the distance to the parent, velocity by the circular speed there")
    {
        Dopri5Tolerance tol;
        tol.rtol = 1e-10;
        tol.atolPos = 1e-6;
        tol.atolVel = 1e-9;
        CHECK(dopri5PositionScale(tol, 7e5) == doctest::Approx(1e-6 + 7e-5).epsilon(1e-15));
        CHECK(dopri5PositionScale(tol, 0.0) == 1e-6);
        // A hundred times farther out, a hundred times looser
        CHECK(dopri5PositionScale(tol, 7e7) / dopri5PositionScale(tol, 7e5) > 90.0);
        CHECK(dopri5VelocityScale(tol, 3.767e12, 7e5) == doctest::Approx(1e-9 + 1e-10 * std::sqrt(3.767e12 / 7e5)).epsilon(1e-15));
        CHECK(dopri5VelocityScale(tol, 0.0, 7e5) == 1e-9);
    }
}
