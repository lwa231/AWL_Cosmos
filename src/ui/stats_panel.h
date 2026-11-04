#pragma once
// Stats window (plan 5.12, 10.3): ImPlot charts of the conservation drift and of the frame time.
//
//   |dE/E0|              energy drift
//   |dP| / sum(mu|v|)    momentum drift against the momentum scale
//   |dL/L0|              angular momentum drift
// all computed from the merge-corrected quantities (Simulation::conservedWithMerges: the logged merge losses are added
// back), so the charts show integrator error only. The reference (E0, P0, L0) is the first sample after reset(); a sim
// time that goes backwards (Stop restoring a snapshot) resets by itself.
//
// The panel keeps its own history and does no physics: the app calls recordConserved() a few times per second and
// recordFrame() once per frame.
#include <cstddef>
#include <vector>
#include "core/diagnostics.h"

namespace cosmos {
namespace ui {

class StatsPanel {
public:
    // One conservation sample at simulation time t (seconds). `momentumScale` is sum(mu_i |v_i|) of the same state.
    void recordConserved(double simTime, const Conserved& c, double momentumScale);
    // One rendered frame: total frame time and the physics part of it, in milliseconds.
    void recordFrame(double frameMs, double physicsMs);
    // Forget the history and the reference (new scene, Start from Stopped).
    void reset();

    // Draws the window if *open. Safe to call every frame.
    void draw(bool* open);

    // Screen rectangle (x0, y0, x1, y1) of the window as of the last draw(); false if it was not drawn.
    bool windowRect(float out[4]) const;

    // Derived values of the latest sample (0 before the first), for tests and readouts.
    struct Drift { double energy = 0.0, momentum = 0.0, angular = 0.0; };
    Drift latest() const { return latest_; }
    std::size_t conservedSamples() const { return t_.size(); }
    std::size_t frameSamples() const { return frameCount_; }

    static constexpr std::size_t kMaxConserved = 2048;   // decimated by 2 when full, so the chart always spans the whole run
    static constexpr std::size_t kFrameHistory = 600;    // frames in the frame-time chart

private:
    static double relative(double delta, double scale);

    // Conservation history (x = simulated days since the first sample), already decimated.
    std::vector<double> t_, dE_, dP_, dL_;
    bool haveReference_ = false;
    Conserved reference_;
    double t0_ = 0.0, lastT_ = 0.0;
    Drift latest_;
    long stride_ = 1;         // keep every stride-th recorded sample
    long recorded_ = 0;

    // Frame-time ring
    std::vector<float> frameX_, frameMs_, physicsMs_;
    std::size_t frameCount_ = 0, frameHead_ = 0;
    double frameIndex_ = 0.0;
    double frameAvg_ = 0.0, frameMax_ = 0.0;
    float rect_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool rectValid_ = false;
};

} // namespace ui
} // namespace cosmos
