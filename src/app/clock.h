#pragma once
// Simulation clock independent of the frame rate (plan section 4.4).
//
// Every real frame the display time moves by  t_target += warp * frameDt.  The clock only decides WHERE the display
// time is; the physics runs on its own grid and is asked to reach t_target (sim_driver.h). Pure logic, no GL.
//
// Rules
//  - frameDt is clamped to [0, maxFrameDt] (default 0.25 s): after a stall (window drag, breakpoint, a slow frame) the
//    simulation does not jump. A NaN or negative frameDt counts as 0.
//  - The warp passed to advance() applies to that call only; whoever changes the warp between frames changes it from
//    the next advance() on, and a frame that is already accounted for is never re-scaled.
//  - While paused advance() adds nothing (the clamp still runs, so lastFrameDt() stays meaningful).
//  - t_target is kept with compensated summation (Kahan-Babuska / Neumaier): sum and a running error term, so adding a
//    small step to a large time does not lose the step to rounding (a million frames at 1/144 s drift by < 1e-9 s
//    where naive summation drifts by ~1e-5 s).
#include <cmath>

namespace cosmos {
namespace app {

class SimClock {
public:
    static constexpr double kDefaultMaxFrameDt = 0.25;   // s, real time

    explicit SimClock(double startTime = 0.0, double maxFrameDt = kDefaultMaxFrameDt)
        : sum_(startTime), maxFrameDt_(maxFrameDt > 0.0 ? maxFrameDt : kDefaultMaxFrameDt)
    {
    }

    // The display time t_target, sim seconds.
    double target() const { return sum_ + comp_; }

    // One real frame of length realDt at the given warp (sim seconds per real second). Returns the sim seconds added
    // (0 while paused). A warp that is negative or not finite counts as 0.
    double advance(double realDt, double warp)
    {
        lastDt_ = clampFrameDt(realDt, maxFrameDt_);
        lastClamped_ = realDt > maxFrameDt_;
        if (paused_ || !(warp > 0.0) || !std::isfinite(warp)) return 0.0;
        const double before = target();
        add(warp * lastDt_);
        return target() - before;
    }

    void setPaused(bool p) { paused_ = p; }
    bool paused() const { return paused_; }

    // Put the display time back to t (only if t is earlier than now; otherwise nothing happens). Used when the physics
    // cannot keep up: t_target waits for it instead of running away. Restarts the compensation.
    void holdAt(double t)
    {
        if (t < target()) reset(t);
    }
    // Set the display time to t whatever it was (new scene, restore of a snapshot).
    void reset(double t)
    {
        sum_ = t;
        comp_ = 0.0;
    }

    // The frame length used by the last advance(), after clamping, and whether the clamp changed it.
    double lastFrameDt() const { return lastDt_; }
    bool lastFrameClamped() const { return lastClamped_; }
    double maxFrameDt() const { return maxFrameDt_; }
    void setMaxFrameDt(double d)
    {
        if (d > 0.0) maxFrameDt_ = d;
    }

    // dt limited to [0, maxDt]; NaN gives 0.
    static double clampFrameDt(double dt, double maxDt)
    {
        if (!(dt > 0.0)) return 0.0;
        return dt < maxDt ? dt : maxDt;
    }

private:
    void add(double x)
    {
        const double t = sum_ + x;
        if (std::fabs(sum_) >= std::fabs(x))
            comp_ += (sum_ - t) + x;
        else
            comp_ += (x - t) + sum_;
        sum_ = t;
    }

    double sum_ = 0.0;
    double comp_ = 0.0;
    double maxFrameDt_ = kDefaultMaxFrameDt;
    double lastDt_ = 0.0;
    bool lastClamped_ = false;
    bool paused_ = false;
};

} // namespace app
} // namespace cosmos
