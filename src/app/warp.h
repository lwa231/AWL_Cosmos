#pragma once
// Time-warp control (plan sections 8 items 5-6, 10.3): the ladder, a continuous slider value, and the memory behind the
// "Resume warp" button. Pure logic, no GL.
//
// Warp is the number of simulated seconds per real second. The ladder is 1, 10, 100, 1e3, 1e4, 1e5, 1e6; the slider
// value can be anything in [0, 1e6] (below 1 is slow motion, 0 stops the clock). Physics never depends on it (4.4).
#include <array>
#include <cstddef>

namespace cosmos {
namespace app {

inline constexpr double kWarpMin = 0.0;
inline constexpr double kWarpMax = 1.0e6;
inline constexpr std::array<double, 7> kWarpLadder = {1.0, 10.0, 100.0, 1.0e3, 1.0e4, 1.0e5, 1.0e6};

class Warp {
public:
    explicit Warp(double initial = 1.0) { set(initial); }

    double value() const { return value_; }

    // Slider / typed value, clamped to [kWarpMin, kWarpMax]. NaN is ignored.
    void set(double w);

    // Next ladder rung above / below the current value (the value need not sit on a rung: 250 steps up to 1000 and
    // down to 100). Up at the top and down at 1 (or below it) change nothing. Return whether the value changed.
    bool stepUp();
    bool stepDown();

    // Index into kWarpLadder when the value is on a rung (relative tolerance 1e-9), else -1.
    int ladderIndex() const;

    // ---- Automatic slow-down and "Resume warp" (plan 8 items 5-6) -------------------------------------------------
    // One level down the ladder for an approaching impact. The first call of a slow-down remembers the warp it started
    // from; further calls (one level each, re-predicting in between) keep that memory. Nothing happens at warp <= 1.
    bool slowDownOneLevel();
    // True when a slow-down was recorded and the warp is still below the remembered value.
    bool canResume() const { return hasSaved_ && saved_ > value_; }
    double resumeValue() const { return hasSaved_ ? saved_ : value_; }
    // Back to the remembered warp (and forget it). Returns whether anything changed.
    bool resume();
    // Forget the memory without changing the warp. Manual changes (set, stepUp, stepDown) keep it while the warp stays
    // below the remembered value, so the user can step up by hand during the slow-down and still press Resume; reaching the
    // remembered value (or more) by hand forgets it. The UI may clear it earlier (M7).
    void clearResume() { hasSaved_ = false; }

    // ---- Slider mapping (M6) ------------------------------------------------------------------------------------
    // Logarithmic slider position in [0, 1] for warp in [1, 1e6]: position = log10(warp) / 6. Warps below 1 map to 0.
    static double sliderFromWarp(double warp);
    // Inverse: warp = 10^(6 position), position clamped to [0, 1], so the slider spans 1 .. 1e6.
    static double warpFromSlider(double position);

private:
    void dropStaleMemory();

    double value_ = 1.0;
    double saved_ = 1.0;
    bool hasSaved_ = false;
};

} // namespace app
} // namespace cosmos
