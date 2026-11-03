#include "core/ias15.h"
#include <algorithm>
#include <cmath>

namespace cosmos {

namespace ias15 {

// Left Gauss-Radau nodes on [0, 1] (8 points, exact quadrature to degree 14), computed at 40 digits.
const double h[8] = {
    0.0,
    0.05626256053692214646565219,
    0.1802406917368923649875799,
    0.3526247171131696373739078,
    0.5471536263305553830014486,
    0.7342101772154105315232106,
    0.8853209468390957680903598,
    0.9775206135612875018911745};

namespace {
// c[m][j] = coefficient of tau^j in tau (tau - h1) ... (tau - h(m-1)); computed once from the nodes.
struct CTable {
    double c[8][8] = {};
    CTable()
    {
        c[1][1] = 1.0;
        for (int m = 2; m < 8; ++m) {
            c[m][m] = 1.0;
            c[m][1] = -h[m - 1] * c[m - 1][1];
            for (int j = 2; j < m; ++j) c[m][j] = c[m - 1][j - 1] - h[m - 1] * c[m - 1][j];
        }
    }
};
const CTable C;
} // namespace

void gFromF(const double F[8], double g[8])
{
    g[0] = 0.0;
    for (int k = 1; k < 8; ++k) {
        double t = (F[k] - F[0]) / h[k];
        for (int i = 1; i < k; ++i) t = (t - g[i]) / (h[k] - h[i]);
        g[k] = t;
    }
}

void bFromG(const double g[8], double b[7])
{
    for (int j = 1; j < 8; ++j) {
        double s = 0.0;
        for (int m = j; m < 8; ++m) s += g[m] * C.c[m][j];
        b[j - 1] = s;
    }
}

double positionAt(double x0, double v0, double a0, const double b[7], double hh, double dt)
{
    double p = b[6] / 72.0;
    p = b[5] / 56.0 + hh * p;
    p = b[4] / 42.0 + hh * p;
    p = b[3] / 30.0 + hh * p;
    p = b[2] / 20.0 + hh * p;
    p = b[1] / 12.0 + hh * p;
    p = b[0] / 6.0 + hh * p;
    p = a0 / 2.0 + hh * p;
    return x0 + v0 * dt * hh + dt * dt * hh * hh * p;
}

double velocityAt(double v0, double a0, const double b[7], double hh, double dt)
{
    double p = b[6] / 8.0;
    p = b[5] / 7.0 + hh * p;
    p = b[4] / 6.0 + hh * p;
    p = b[3] / 5.0 + hh * p;
    p = b[2] / 4.0 + hh * p;
    p = b[1] / 3.0 + hh * p;
    p = b[0] / 2.0 + hh * p;
    p = a0 + hh * p;
    return v0 + dt * hh * p;
}

double accelerationAt(double a0, const double b[7], double hh)
{
    double p = b[6];
    p = b[5] + hh * p;
    p = b[4] + hh * p;
    p = b[3] + hh * p;
    p = b[2] + hh * p;
    p = b[1] + hh * p;
    p = b[0] + hh * p;
    return a0 + hh * p;
}

} // namespace ias15

namespace {

// Kahan summation: sum += inc, with the rounding error of the addition kept in c (true sum = sum - c).
inline void kahanAdd(double& sum, double& c, double inc)
{
    const double y = inc - c;
    const double t = sum + y;
    c = (t - sum) - y;
    sum = t;
}

const double INF = std::numeric_limits<double>::infinity();

// Largest tau (in units of the source step) a predictor polynomial is extrapolated to; a normal step growth of 4x reaches 5.
const double MAX_PREDICTOR_TAU = 8.0;

} // namespace

// Stage 1 (plan 6.3): fixed step, Jacobi rounds, b restarted from 0 each step.
int stepIAS15Fixed(BodyState& s, const std::vector<double>& mu, Accel& a, double dt)
{
    const std::size_t n = s.x.size();
    const std::size_t K = 3 * n;
    if (n == 0) return 0;

    std::vector<double> x0(K), v0(K), a0(K);
    double a0max = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        x0[3 * i] = s.x[i];  x0[3 * i + 1] = s.y[i];  x0[3 * i + 2] = s.z[i];
        v0[3 * i] = s.vx[i]; v0[3 * i + 1] = s.vy[i]; v0[3 * i + 2] = s.vz[i];
        a0[3 * i] = a.ax[i]; a0[3 * i + 1] = a.ay[i]; a0[3 * i + 2] = a.az[i];
    }
    for (std::size_t k = 0; k < K; ++k) a0max = std::max(a0max, std::abs(a0[k]));

    if (a0max == 0.0) { // nothing accelerates anything: straight-line drift, a stays zero
        for (std::size_t i = 0; i < n; ++i) {
            s.x[i] += s.vx[i] * dt;
            s.y[i] += s.vy[i] * dt;
            s.z[i] += s.vz[i] * dt;
        }
        return 0;
    }

    std::vector<double> b(7 * K, 0.0), g(8 * K, 0.0), F(8 * K, 0.0);
    BodyState node;
    node.x.resize(n); node.y.resize(n); node.z.resize(n);
    node.vx = s.vx; node.vy = s.vy; node.vz = s.vz; // gravity is position-only: velocities are not read
    Accel an;

    int rounds = 0;
    while (rounds < 25) {
        ++rounds;
        // all seven node accelerations from the polynomial of the previous round (Jacobi style)
        double amax = a0max;
        for (int m = 1; m < 8; ++m) {
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t k = 3 * i;
                node.x[i] = ias15::positionAt(x0[k],     v0[k],     a0[k],     &b[7 * k],       ias15::h[m], dt);
                node.y[i] = ias15::positionAt(x0[k + 1], v0[k + 1], a0[k + 1], &b[7 * (k + 1)], ias15::h[m], dt);
                node.z[i] = ias15::positionAt(x0[k + 2], v0[k + 2], a0[k + 2], &b[7 * (k + 2)], ias15::h[m], dt);
            }
            computeAccelerations(node, mu, an);
            for (std::size_t i = 0; i < n; ++i) {
                F[8 * (3 * i)     + m] = an.ax[i];
                F[8 * (3 * i + 1) + m] = an.ay[i];
                F[8 * (3 * i + 2) + m] = an.az[i];
                amax = std::max(amax, std::max(std::abs(an.ax[i]), std::max(std::abs(an.ay[i]), std::abs(an.az[i]))));
            }
        }
        double db6 = 0.0;
        for (std::size_t k = 0; k < K; ++k) {
            F[8 * k] = a0[k];
            const double g7old = g[8 * k + 7];
            ias15::gFromF(&F[8 * k], &g[8 * k]);
            ias15::bFromG(&g[8 * k], &b[7 * k]);
            db6 = std::max(db6, std::abs(g[8 * k + 7] - g7old));
        }
        if (db6 <= 1e-16 * amax) break;
    }

    for (std::size_t k = 0; k < K; ++k) {
        const double dx = ias15::positionAt(0.0, v0[k], a0[k], &b[7 * k], 1.0, dt);
        const double dv = ias15::velocityAt(0.0, a0[k], &b[7 * k], 1.0, dt);
        const std::size_t i = k / 3;
        switch (k % 3) {
        case 0: s.x[i] = x0[k] + dx; s.vx[i] = v0[k] + dv; break;
        case 1: s.y[i] = x0[k] + dx; s.vy[i] = v0[k] + dv; break;
        default: s.z[i] = x0[k] + dx; s.vz[i] = v0[k] + dv; break;
        }
    }
    computeAccelerations(s, mu, a);
    return rounds;
}

// ---------------------------------------------------------------------------------------------------------------
// Adaptive IAS15 (plan 6.3, stage 2)
// ---------------------------------------------------------------------------------------------------------------

void IAS15::reset()
{
    n_ = 0; // arrays are re-sized (and the carries zeroed) on the next step
    haveA_ = false;
    aCur_.ax.clear();
    aCur_.ay.clear();
    aCur_.az.clear();
    haveHistory_ = false;
    dtLast_ = 0.0;
    dtNext_ = 0.0;
    dtPrev_ = 0.0;
    tCarry_ = 0.0;
    tExpected_ = std::numeric_limits<double>::quiet_NaN();
}

void IAS15::ensureSize(std::size_t n)
{
    // a hint from setNextDt() survives the first sizing after a reset, but not a change in the number of bodies
    const double hint = n_ == 0 ? dtNext_ : 0.0;
    reset();
    dtNext_ = hint;
    n_ = n;
    const std::size_t K = 3 * n;
    x0_.assign(K, 0.0); v0_.assign(K, 0.0); a0_.assign(K, 0.0);
    cx_.assign(K, 0.0); cv_.assign(K, 0.0);
    xd_.assign(K, 0.0); vd_.assign(K, 0.0); ad_.assign(K, 0.0);
    b_.assign(7 * K, 0.0); bt_.assign(7 * K, 0.0); bf_.assign(7 * K, 0.0);
    gt_.assign(8 * K, 0.0);
    bPrev_.assign(7 * K, 0.0);
    node_.x.assign(n, 0.0); node_.y.assign(n, 0.0); node_.z.assign(n, 0.0);
    node_.vx.assign(n, 0.0); node_.vy.assign(n, 0.0); node_.vz.assign(n, 0.0);
}

// First step: 1% of the shortest dynamical time. For each body i with its nearest neighbour j at distance r, the
// free-fall time sqrt(r / |a_i|) and the crossing time r / |v_i - v_j|. The controller fixes the rest within a few steps.
double IAS15::initialStep(const BodyState& s) const
{
    double best = INF;
    for (std::size_t i = 0; i < n_; ++i) {
        const double ai = std::sqrt((aCur_.ax[i] * aCur_.ax[i] + aCur_.ay[i] * aCur_.ay[i]) + aCur_.az[i] * aCur_.az[i]);
        if (!(ai > 0.0)) continue;
        double r2 = INF;
        std::size_t jn = i;
        for (std::size_t j = 0; j < n_; ++j) {
            if (j == i) continue;
            const double dx = s.x[j] - s.x[i], dy = s.y[j] - s.y[i], dz = s.z[j] - s.z[i];
            const double d2 = (dx * dx + dy * dy) + dz * dz;
            if (d2 > 0.0 && d2 < r2) { r2 = d2; jn = j; }
        }
        if (jn == i) {
            // no neighbour (a single body in an external field): use its distance from the origin and its speed
            const double r = std::sqrt((s.x[i] * s.x[i] + s.y[i] * s.y[i]) + s.z[i] * s.z[i]);
            if (r > 0.0) best = std::min(best, std::sqrt(r / ai));
            const double v = std::sqrt((s.vx[i] * s.vx[i] + s.vy[i] * s.vy[i]) + s.vz[i] * s.vz[i]);
            if (v > 0.0) best = std::min(best, v / ai);
            continue;
        }
        const double r = std::sqrt(r2);
        best = std::min(best, std::sqrt(r / ai));
        const double wx = s.vx[i] - s.vx[jn], wy = s.vy[i] - s.vy[jn], wz = s.vz[i] - s.vz[jn];
        const double w = std::sqrt((wx * wx + wy * wy) + wz * wz);
        if (w > 0.0) best = std::min(best, r / w);
    }
    return std::isfinite(best) ? 0.01 * best : 0.0;
}

// Target precision for this step: params.epsilon, raised when the roundoff floor of the force (see IAS15Params) is higher.
double IAS15::effectiveEpsilon(const BodyState& s) const
{
    if (!(params.noiseFactor > 0.0)) return params.epsilon;
    double amax = 0.0;
    std::vector<double> an(n_), pos(n_);
    for (std::size_t i = 0; i < n_; ++i) {
        an[i] = std::sqrt((aCur_.ax[i] * aCur_.ax[i] + aCur_.ay[i] * aCur_.ay[i]) + aCur_.az[i] * aCur_.az[i]);
        pos[i] = std::max(std::abs(s.x[i]), std::max(std::abs(s.y[i]), std::abs(s.z[i])));
        amax = std::max(amax, an[i]);
    }
    if (!(amax > 0.0)) return params.epsilon;
    double floorRel = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        for (std::size_t j = i + 1; j < n_; ++j) {
            const double dx = s.x[j] - s.x[i], dy = s.y[j] - s.y[i], dz = s.z[j] - s.z[i];
            const double d2 = (dx * dx + dy * dy) + dz * dz;
            if (!(d2 > 0.0)) continue;
            const double u = std::numeric_limits<double>::epsilon() * std::max(pos[i], pos[j]);
            const double w = std::min(1.0, std::max(an[i], an[j]) / amax); // the pair's share of the largest acceleration
            floorRel = std::max(floorRel, w * u / std::sqrt(d2));
        }
    }
    const double e = std::max(params.epsilon, params.noiseFactor * floorRel);
    return std::min(e, params.epsilon * params.maxEpsilonBoost);
}

void IAS15::predict(const std::vector<double>& bSrc, double q, double off)
{
    const std::size_t K = 3 * n_;
    for (std::size_t k = 0; k < K; ++k) {
        const double* b = &bSrc[7 * k];
        // the constant a0 of the old polynomial cancels in the divided differences, so leave it out
        double F[8];
        for (int m = 0; m < 8; ++m) {
            const double tau = off + q * ias15::h[m];
            double p = b[6];
            p = b[5] + tau * p;
            p = b[4] + tau * p;
            p = b[3] + tau * p;
            p = b[2] + tau * p;
            p = b[1] + tau * p;
            p = b[0] + tau * p;
            F[m] = tau * p;
        }
        ias15::gFromF(F, &gt_[8 * k]);
        ias15::bFromG(&gt_[8 * k], &bt_[7 * k]);
    }
}

bool IAS15::iterate(const ForceFn& force, double dt, int& iters, double& amax, double& rel)
{
    const std::size_t n = n_, K = 3 * n_;
    double a0max = 0.0;
    for (std::size_t k = 0; k < K; ++k) a0max = std::max(a0max, std::abs(a0_[k]));

    bool converged = false;
    rel = 0.0;
    iters = 0;
    amax = a0max;
    while (iters < params.maxIterations) {
        ++iters;
        double db6 = 0.0;
        amax = a0max;
        for (int m = 1; m < 8; ++m) {
            const double hh = ias15::h[m];
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t k = 3 * i;
                node_.x[i]  = ias15::positionAt(x0_[k],     v0_[k],     a0_[k],     &bt_[7 * k],       hh, dt);
                node_.y[i]  = ias15::positionAt(x0_[k + 1], v0_[k + 1], a0_[k + 1], &bt_[7 * (k + 1)], hh, dt);
                node_.z[i]  = ias15::positionAt(x0_[k + 2], v0_[k + 2], a0_[k + 2], &bt_[7 * (k + 2)], hh, dt);
                node_.vx[i] = ias15::velocityAt(v0_[k],     a0_[k],     &bt_[7 * k],       hh, dt);
                node_.vy[i] = ias15::velocityAt(v0_[k + 1], a0_[k + 1], &bt_[7 * (k + 1)], hh, dt);
                node_.vz[i] = ias15::velocityAt(v0_[k + 2], a0_[k + 2], &bt_[7 * (k + 2)], hh, dt);
            }
            force(node_, fn_);
            ++forceCalls_;
            const double* Fc[3] = {fn_.ax.data(), fn_.ay.data(), fn_.az.data()};
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t c = 0; c < 3; ++c) {
                    const std::size_t k = 3 * i + c;
                    const double F = Fc[c][i];
                    amax = std::max(amax, std::abs(F));
                    // Gauss-Seidel: g_m from the new F and the already-updated g_1..g_(m-1), then b follows at once
                    double* g = &gt_[8 * k];
                    double t = (F - a0_[k]) / hh;
                    for (int j = 1; j < m; ++j) t = (t - g[j]) / (hh - ias15::h[j]);
                    const double dg = t - g[m];
                    g[m] = t;
                    double* b = &bt_[7 * k];
                    for (int j = 1; j <= m; ++j) b[j - 1] += dg * ias15::C.c[m][j];
                    if (m == 7) db6 = std::max(db6, std::abs(dg));
                }
            }
        }
        rel = amax > 0.0 ? db6 / amax : 0.0;
        if (!(rel > params.epsilonPC)) { converged = true; break; }
    }
    // rebuild b from g once: removes the drift of the incremental updates
    for (std::size_t k = 0; k < K; ++k) ias15::bFromG(&gt_[8 * k], &bt_[7 * k]);
    return converged;
}

double IAS15::step(BodyState& s, const ForceFn& force, double& t, double tMax)
{
    lastRejections_ = 0;
    const std::size_t n = s.x.size();
    if (n == 0 || !(t < tMax)) return 0.0;
    if (n != n_) ensureSize(n);
    const std::size_t K = 3 * n;

    if (!(t == tExpected_)) tCarry_ = 0.0;

    if (!haveA_) {
        force(s, aCur_);
        ++forceCalls_;
        haveA_ = true;
    }
    for (std::size_t i = 0; i < n; ++i) {
        x0_[3 * i] = s.x[i];  x0_[3 * i + 1] = s.y[i];  x0_[3 * i + 2] = s.z[i];
        v0_[3 * i] = s.vx[i]; v0_[3 * i + 1] = s.vy[i]; v0_[3 * i + 2] = s.vz[i];
        a0_[3 * i] = aCur_.ax[i]; a0_[3 * i + 1] = aCur_.ay[i]; a0_[3 * i + 2] = aCur_.az[i];
    }

    double dtTry = dtNext_;
    if (!(dtTry > 0.0)) dtTry = initialStep(s);
    if (!(dtTry > 0.0)) dtTry = std::isfinite(tMax) ? tMax - t : 1.0; // nothing accelerates: any step is exact
    dtTry = std::min(dtTry, params.dtMax);

    const double epsEff = effectiveEpsilon(s);
    lastEpsilon_ = epsEff;

    const std::vector<double>* src = haveHistory_ ? &b_ : nullptr;
    double srcDt = dtLast_, off = 1.0;

    double dt = 0.0, ratio = 0.0, dtFree = dtTry;
    bool clipped = false;
    int iters = 0;
    bool converged = true;
    double lastErrAttempt = 0.0;
    double lastRelAttempt = 0.0;
    for (int attempt = 0;; ++attempt) {
        dtFree = dtTry;
        dt = dtTry;
        clipped = false;
        const double remaining = tMax - t;
        if (dt >= remaining) { dt = remaining; clipped = true; }

        if (attempt == 0 && haveHistory_) {
            // Extrapolating a polynomial to tau = off + dt / srcDt costs accuracy roughly as tau^7, so after a very short
            // step (clipped by tMax) fall back on the step before it, or on b = 0 if neither is good enough.
            const double tauLast = 1.0 + dt / dtLast_;
            double tauPrev = INF, offPrev = 1.0;
            if (dtPrev_ > 0.0) {
                offPrev = (dtPrev_ + dtLast_) / dtPrev_;
                tauPrev = offPrev + dt / dtPrev_;
            }
            if (std::min(tauLast, tauPrev) > MAX_PREDICTOR_TAU) {
                src = nullptr;
            } else if (tauPrev < tauLast) {
                src = &bPrev_; srcDt = dtPrev_; off = offPrev;
            }
        }

        if (src) {
            predict(*src, dt / srcDt, off);
        } else {
            std::fill(bt_.begin(), bt_.end(), 0.0);
            std::fill(gt_.begin(), gt_.end(), 0.0);
        }

        double amax = 0.0, rel = 0.0;
        converged = iterate(force, dt, iters, amax, rel);
        lastRelAttempt = rel;
        totalIterations_ += iters;

        double b6max = 0.0;
        for (std::size_t k = 0; k < K; ++k) b6max = std::max(b6max, std::abs(bt_[7 * k + 6]));
        const bool finite = std::isfinite(b6max) && std::isfinite(amax);
        lastErrAttempt = (finite && amax > 0.0) ? b6max / amax : 0.0;
        if (!finite) ratio = 0.0;
        else if (b6max == 0.0 || amax == 0.0) ratio = INF;
        else ratio = std::pow(epsEff / (b6max / amax), 1.0 / 7.0);

        // an iteration that is still moving b6 by more than 10 epsilon when it stops has not found the polynomial at all
        const bool broken = !finite || (!converged && rel > 10.0 * epsEff);
        const bool canShrink = dt > params.dtMin && attempt < 60 && (t + dt) != t;
        if (canShrink && (broken || ratio < params.safety)) {
            ++lastRejections_;
            ++totalRejections_;
            if (!broken) {
                // retry the same step with the failed polynomial (re-expressed on the shorter step) as predictor
                bf_ = bt_;
                src = &bf_;
                srcDt = dt;
                off = 0.0;
                dtTry = std::max(dt * ratio, params.dtMin);
            } else {
                src = nullptr;
                dtTry = std::max(dt * params.safety, params.dtMin);
            }
            continue;
        }
        break;
    }

    // accept ---------------------------------------------------------------------------------------------------
    bPrev_ = b_;
    dtPrev_ = haveHistory_ ? dtLast_ : 0.0;
    xd_ = x0_; vd_ = v0_; ad_ = a0_;
    b_ = bt_;
    dtLast_ = dt;
    tStart_ = t;
    haveHistory_ = true;
    lastIterations_ = iters;
    lastConverged_ = converged;
    lastError_ = lastErrAttempt;
    lastResidual_ = lastRelAttempt;
    ++totalSteps_;

    for (std::size_t k = 0; k < K; ++k) {
        const double dx = ias15::positionAt(0.0, v0_[k], a0_[k], &b_[7 * k], 1.0, dt);
        const double dv = ias15::velocityAt(0.0, a0_[k], &b_[7 * k], 1.0, dt);
        kahanAdd(x0_[k], cx_[k], dx);
        kahanAdd(v0_[k], cv_[k], dv);
    }
    for (std::size_t i = 0; i < n; ++i) {
        s.x[i] = x0_[3 * i];  s.y[i] = x0_[3 * i + 1];  s.z[i] = x0_[3 * i + 2];
        s.vx[i] = v0_[3 * i]; s.vy[i] = v0_[3 * i + 1]; s.vz[i] = v0_[3 * i + 2];
    }

    if (clipped) {
        t = tMax;
        tCarry_ = 0.0;
    } else {
        kahanAdd(t, tCarry_, dt);
    }
    tEnd_ = t;
    tExpected_ = t;

    force(s, aCur_);
    ++forceCalls_;

    // next step size: growth limited to 1 / safety; a step clipped by tMax does not shrink the proposal
    double prop = dt * std::min(ratio, 1.0 / params.safety);
    prop = std::min(std::max(prop, params.dtMin), params.dtMax);
    if (clipped && !(prop < dt)) prop = std::min(std::max(prop, dtFree), params.dtMax);
    dtNext_ = prop;
    return dt;
}

Vec3 IAS15::positionOf(std::size_t i, double tau) const
{
    const std::size_t k = 3 * i;
    return {ias15::positionAt(xd_[k],     vd_[k],     ad_[k],     &b_[7 * k],       tau, dtLast_),
            ias15::positionAt(xd_[k + 1], vd_[k + 1], ad_[k + 1], &b_[7 * (k + 1)], tau, dtLast_),
            ias15::positionAt(xd_[k + 2], vd_[k + 2], ad_[k + 2], &b_[7 * (k + 2)], tau, dtLast_)};
}

Vec3 IAS15::velocityOf(std::size_t i, double tau) const
{
    const std::size_t k = 3 * i;
    return {ias15::velocityAt(vd_[k],     ad_[k],     &b_[7 * k],       tau, dtLast_),
            ias15::velocityAt(vd_[k + 1], ad_[k + 1], &b_[7 * (k + 1)], tau, dtLast_),
            ias15::velocityAt(vd_[k + 2], ad_[k + 2], &b_[7 * (k + 2)], tau, dtLast_)};
}

Vec3 IAS15::accelerationOf(std::size_t i, double tau) const
{
    const std::size_t k = 3 * i;
    return {ias15::accelerationAt(ad_[k],     &b_[7 * k],       tau),
            ias15::accelerationAt(ad_[k + 1], &b_[7 * (k + 1)], tau),
            ias15::accelerationAt(ad_[k + 2], &b_[7 * (k + 2)], tau)};
}

void IAS15::stateAt(double tau, BodyState& out) const
{
    out.x.resize(n_); out.y.resize(n_); out.z.resize(n_);
    out.vx.resize(n_); out.vy.resize(n_); out.vz.resize(n_);
    for (std::size_t i = 0; i < n_; ++i) {
        const Vec3 p = positionOf(i, tau), v = velocityOf(i, tau);
        out.x[i] = p.x; out.y[i] = p.y; out.z[i] = p.z;
        out.vx[i] = v.x; out.vy[i] = v.y; out.vz[i] = v.z;
    }
}

void IAS15::accelerationsAt(double tau, Accel& out) const
{
    out.ax.resize(n_); out.ay.resize(n_); out.az.resize(n_);
    for (std::size_t i = 0; i < n_; ++i) {
        const Vec3 a = accelerationOf(i, tau);
        out.ax[i] = a.x; out.ay[i] = a.y; out.az[i] = a.z;
    }
}

} // namespace cosmos
