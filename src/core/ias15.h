#pragma once
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>
#include "core/bodies.h"
#include "core/gravity.h"
#include "core/vec3.h"

namespace cosmos {

namespace ias15 {

// Gauss-Radau nodes in [0, 1): h[0] = 0, h[1..7] are the roots used inside one step (tau = t / dt).
extern const double h[8];

// Divided differences: F[0..7] are the accelerations at the nodes (F[0] = a0), g[1..7] are filled (g[0] unused).
void gFromF(const double F[8], double g[8]);

// Convert g[1..7] to the monomial coefficients b[0..6] of a(tau) = a0 + b0*tau + b1*tau^2 + ... + b6*tau^7.
void bFromG(const double g[8], double b[7]);

// Position / velocity of one component at tau = hh (0..1) inside a step of length dt, from the step-start values
// x0, v0, a0 and the polynomial coefficients b[0..6]. hh = 1 gives the end of the step.
double positionAt(double x0, double v0, double a0, const double b[7], double hh, double dt);
double velocityAt(double v0, double a0, const double b[7], double hh, double dt);

// Acceleration of one component at tau = hh, a0 + b0*hh + ... + b6*hh^7.
double accelerationAt(double a0, const double b[7], double hh);

} // namespace ias15

// One fixed-step IAS15 step (plan section 6.3, stage 1: no step-size control, used to check the convergence order).
// Same contract for a as stepKDK: valid on entry, valid at the new positions on return. b starts at 0 every step and
// is refined by Jacobi-style predictor-corrector rounds (all seven nodes from the previous round's polynomial) until
// max|change of b6| / max|a| < 1e-16, at most 25 rounds. Hitting the cap at the roundoff floor is normal.
// Returns the number of rounds used (0 if all accelerations are zero: the bodies just drift).
int stepIAS15Fixed(BodyState& s, const std::vector<double>& mu, Accel& a, double dt);

// Force callback of the adaptive integrator: fill a (ax, ay, az, one entry per body) with the accelerations of the
// state s. It must resize/overwrite a, like computeAccelerations does. The velocities in s are the velocities at the
// node time, so velocity-dependent forces work too. Called ~8-30 times per step, so keep it cheap.
using ForceFn = std::function<void(const BodyState&, Accel&)>;

struct IAS15Params {
    // Step-size precision eps_b of the paper: the step is chosen so that max|b6| / max|a| ~ epsilon, where b6 is the
    // 7th coefficient of the acceleration polynomial (in acceleration units) and max|a| is taken over all bodies and
    // components, over the step's start and its seven nodes. 1e-9 is the paper's default and keeps the energy error
    // near machine precision. The step scales as epsilon^(1/7).
    double epsilon = 1e-9;
    // Predictor-corrector convergence: stop when max|change of b6| / max|a| < epsilonPC (paper: 1e-16).
    double epsilonPC = 1e-16;
    int maxIterations = 12;
    // A step is rejected and retried when the proposed step is smaller than safety * dt (a shrink of more than 4x), or
    // when the iteration is still changing b6 by more than 10 epsilon at the cap. Milder overshoots are accepted, as in
    // the paper. The step grows by at most 1 / safety (4x) from one step to the next.
    double safety = 0.25;
    // Bounds on the steps the controller proposes (clipping to tMax can still go below dtMin).
    double dtMin = 0.0;
    double dtMax = std::numeric_limits<double>::infinity();
    // Roundoff guard (not in the paper). Positions are rounded to ulp(|x|), so the force between two bodies a distance
    // r apart is only known to about eps_mach * |x| / r, and b6 (a 7th divided difference) amplifies that by 10^2..10^3.
    // When this floor rises above epsilon (a tight pair far from the origin) no step, however small, gets max|b6|/max|a|
    // down to epsilon and the step collapses towards zero. So the target is raised to noiseFactor * (that floor), but
    // never by more than a factor maxEpsilonBoost. Set noiseFactor = 0 to follow the paper exactly.
    double noiseFactor = 1e4;
    double maxEpsilonBoost = 1e5;
};

// Adaptive IAS15 (Rein & Spiegel 2015, MNRAS 446, 1424), plan section 6.3.
//   * 15th-order Gauss-Radau quadrature with seven nodes; a(tau) inside a step is the polynomial
//     a0 + b0 tau + ... + b6 tau^7 (tau = (t - t0) / dt, coefficients in acceleration units).
//   * Gauss-Seidel predictor-corrector: the node accelerations update b as soon as they are computed.
//   * The predictor of a step is the previous step's polynomial extrapolated by the step-size ratio, so a step
//     usually takes 2-3 iterations (7 force calls each) plus one force call at the new positions.
//   * Positions and velocities are summed with Kahan compensation; the carry lives in this object.
//   * Global step-size control from max|b6| / max|a| (see IAS15Params); first step chosen automatically.
//   * Dense output: positions, velocities and accelerations of every body anywhere in the last accepted step.
// The object owns the cached acceleration at the current state, the predictor history and the dense output. Call
// reset() after anything that changes the bodies behind its back (merge, add, remove, edit).
// Deterministic: no global state, fixed summation order.
class IAS15 {
public:
    explicit IAS15(const IAS15Params& p = IAS15Params()) : params(p) {}

    IAS15Params params;

    // Drop the predictor history, dense output, cached accelerations, Kahan carries and the step-size proposal.
    // The next step recomputes a and picks its first step size automatically (1% of the shortest dynamical time).
    // Statistics counters are kept. Call this after a merge, after adding/removing/editing bodies (a change in the
    // number of bodies is also noticed by step() on its own); without it the cached accelerations are stale and the
    // first step collapses to a tiny size.
    void reset();

    // Optional hint for the next step size: replaces the automatic first step or the controller's proposal once.
    // It survives a following reset()-then-step, so call it after reset().
    void setNextDt(double dt) { dtNext_ = dt; }

    // Advance the state s from time t by one adaptive step and return the step taken. The step never goes past tMax:
    // it is clipped to land exactly on tMax, and t is then set to exactly tMax (the controller's next proposal is not
    // reduced by the clip). If t >= tMax or s is empty nothing happens and 0 is returned. On return t is the new time,
    // summed with Kahan compensation (the carry is kept here and used as long as t is the value returned by the
    // previous call). Rejected attempts happen inside this call (see lastRejections()); s only ever holds accepted
    // states. A new first step costs a few extra iterations.
    // Non-finite numbers are never committed. If the state or its accelerations at the start are not finite (for
    // example two coincident bodies, or a state that is already NaN) nothing is attempted. If a trial step produces a
    // non-finite force or polynomial, it is rejected and retried with a quarter of the step (a force that is NaN only
    // for the larger step is thereby cured); if the retries run out (60 attempts, dtMin, or a step too small to move
    // t) the step is abandoned. In both cases step() returns 0, leaves s, t and the integrator's history and dense
    // output untouched, and lastFailed() is true. Calling step() again tries a shorter step (see nextDt()).
    // force is called with node states of s's size. If the number of bodies changed since the last call, the history
    // is dropped as by reset().
    double step(BodyState& s, const ForceFn& force, double& t,
                double tMax = std::numeric_limits<double>::infinity());

    // The accelerations at the state s the last step() left (the force call that primes the next step); empty before the
    // first step and after reset(). Valid only as long as s is not changed behind the integrator's back.
    const Accel& currentAccelerations() const { return aCur_; }

    // Statistics ----------------------------------------------------------------------------------------------
    double lastDt() const { return dtLast_; }                   // length of the last accepted step (0 before any)
    double nextDt() const { return dtNext_; }                   // size the controller proposes for the next step
    double lastError() const { return lastError_; }              // max|b6| / max|a| of the last accepted step (the controller aims at epsilon)
    double lastEpsilon() const { return lastEpsilon_; }          // precision target used by the last step() call (epsilon, or more if roundoff-limited)
    double lastResidual() const { return lastResidual_; }        // max|change of b6| / max|a| in the last iteration of the last accepted attempt
    int lastIterations() const { return lastIterations_; }      // predictor-corrector iterations of the last accepted attempt
    int lastRejections() const { return lastRejections_; }      // attempts rejected during the last step() call
    // true if the last step() call could not produce a finite step and returned 0 without advancing (see step()). It is
    // reset at the start of every step() call: false after a successful step and after a call that did nothing because
    // t >= tMax or s was empty. Check it (or the returned 0) before looping on step() again.
    bool lastFailed() const { return lastFailed_; }
    long totalSteps() const { return totalSteps_; }
    long totalRejections() const { return totalRejections_; }
    long totalIterations() const { return totalIterations_; }
    long forceCalls() const { return forceCalls_; }
    // false if the iteration stopped at maxIterations before the b6 change fell below epsilonPC. Residuals of 1e-13 or
    // so (measured, in a few steps in a thousand) are the roundoff floor of the force and harmless.
    bool lastConverged() const { return lastConverged_; }

    // Dense output of the last accepted step -----------------------------------------------------------------
    // The last accepted step as a polynomial: tau = (time - stepStartTime()) / lastDt(), valid for tau in [0, 1]
    // (slightly outside extrapolates the polynomial). Position is 9th degree in tau, velocity 8th, acceleration 7th;
    // tau = 0 reproduces the state at the start of the step exactly, tau = 1 the end to roundoff. The queries are
    // const, need no force call and do not allocate (except the whole-system ones, which resize their outputs), and
    // stay valid until the next step() or reset(). Do not call them before hasDenseOutput() is true.
    bool hasDenseOutput() const { return dtLast_ > 0.0; }
    double stepStartTime() const { return tStart_; }
    double stepEndTime() const { return tEnd_; }
    double tauAt(double time) const { return (time - tStart_) / dtLast_; }
    std::size_t bodyCount() const { return n_; }

    Vec3 positionOf(std::size_t i, double tau) const;
    Vec3 velocityOf(std::size_t i, double tau) const;
    Vec3 accelerationOf(std::size_t i, double tau) const;

    void stateAt(double tau, BodyState& out) const;      // x, y, z, vx, vy, vz of every body
    void accelerationsAt(double tau, Accel& out) const;

private:
    // Fill bt_ (the b of the trial step) by evaluating the polynomial bSrc, which belongs to a step of length srcDt,
    // at tau_old = off + q * h[k] (q = dt / srcDt; off = 1 continues after the step, off = 0 restarts the same step).
    void predict(const std::vector<double>& bSrc, double q, double off);
    // Gauss-Seidel iterations on the trial step; returns false if the iteration cap was hit above the floor. finite is
    // set to false (and the iteration abandoned) as soon as a node acceleration is not finite.
    bool iterate(const ForceFn& force, double dt, int& iters, double& amax, double& rel, bool& finite);
    double initialStep(const BodyState& s) const;
    double effectiveEpsilon(const BodyState& s);
    void ensureSize(std::size_t n);

    std::size_t n_ = 0;                 // bodies
    // Phase space at the start of the current step, flat: index 3 * body + component.
    std::vector<double> x0_, v0_, a0_;
    std::vector<double> cx_, cv_;       // Kahan carries
    Accel aCur_;                        // accelerations at the current state
    bool haveA_ = false;

    // Last accepted step (dense output and predictor source)
    std::vector<double> xd_, vd_, ad_;  // x0, v0, a0 of that step
    std::vector<double> b_;             // 7 coefficients per component
    double dtLast_ = 0.0;
    std::vector<double> bPrev_;         // the step before the last one (predictor fallback after a very short step)
    double dtPrev_ = 0.0;
    double tStart_ = 0.0, tEnd_ = 0.0;
    bool haveHistory_ = false;

    // Trial step scratch
    std::vector<double> bt_, gt_, bf_;  // bt_: 7 per component, gt_: 8 per component, bf_: copy of a failed attempt's b
    BodyState node_;
    Accel fn_;
    std::vector<double> posScale_, accMag_; // scratch of effectiveEpsilon(), one entry per body

    double dtNext_ = 0.0;
    double tCarry_ = 0.0, tExpected_ = std::numeric_limits<double>::quiet_NaN();
    double lastError_ = 0.0;
    double lastEpsilon_ = 0.0;
    double lastResidual_ = 0.0;
    int lastIterations_ = 0, lastRejections_ = 0;
    long totalSteps_ = 0, totalRejections_ = 0, totalIterations_ = 0, forceCalls_ = 0;
    bool lastConverged_ = true;
    bool lastFailed_ = false;
};

} // namespace cosmos
