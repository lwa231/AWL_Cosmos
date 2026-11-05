#pragma once
// Time-warp control (plan sections 8 items 5-6, 10.3): the ladder, a continuous slider value, and the memory behind the
// "Resume warp" button. Pure logic, no GL.
//
// Warp is the number of simulated seconds per real second. The ladder (the "stops" of V1.1 step 2) is
//   1, 2, 5, 10, 30 s;  1, 2, 5, 10, 30 min;  1, 2, 6, 12 h;  1, 2, 5, 10 days   of simulated time per real second.
// The slider spans the stops (1 .. 864000) and snaps to them; a typed or scripted value can be anything in [0, kWarpMax]
// (below 1 is slow motion, 0 stops the clock; kWarpMax stays 1e6 so scripted runs and benchmarks keep their range, the UI
// tops out at the last stop). Physics never depends on it (4.4).
#include <array>
#include <cstddef>
#include <string>

namespace cosmos {
namespace app {

inline constexpr double kWarpMin = 0.0;
inline constexpr double kWarpMax = 1.0e6;
inline constexpr std::array<double, 18> kWarpLadder = {1.0,    2.0,    5.0,     10.0,    30.0,    60.0,
                                                       120.0,  300.0,  600.0,   1800.0,  3600.0,  7200.0,
                                                       21600.0, 43200.0, 86400.0, 172800.0, 432000.0, 864000.0};
inline constexpr double kWarpTopStop = 864000.0;   // the last stop: the right end of the slider (10 days per second)

// The stop nearest to `warp` on the logarithmic scale of the slider (geometric midpoints between neighbours). Values at or below
// the first stop give 1, values above the last give the last. NaN gives 1.
double nearestStop(double warp);

// The warp as a short readout for the slider: "30 s/s", "2 min/s", "6 h/s", "5 d/s"; a value between stops keeps three
// significant figures without trailing zeros ("1.37 h/s", "0.5 s/s"). 0 is "0 s/s"; non-finite input "-- /s".
std::string formatWarp(double warp);

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

    // ---- Slider mapping (M6, V1.1 step 2) --------------------------------------------------------------------------
    // Logarithmic slider position in [0, 1] for warp in [1, kWarpTopStop]: position = ln(warp) / ln(kWarpTopStop). Warps below
    // 1 map to 0, above the last stop to 1.
    static double sliderFromWarp(double warp);
    // Inverse: warp = kWarpTopStop^position, position clamped to [0, 1], so the slider spans 1 .. 864000.
    static double warpFromSlider(double position);
    // What a slider drag sets: the nearest stop to warpFromSlider(position), or the smooth value when `smooth` (Ctrl held).
    static double warpFromSliderDrag(double position, bool smooth);

private:
    void dropStaleMemory();

    double value_ = 1.0;
    double saved_ = 1.0;
    bool hasSaved_ = false;
};

} // namespace app
} // namespace cosmos
