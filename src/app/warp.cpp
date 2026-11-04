#include "app/warp.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

namespace {
constexpr double kRungTol = 1.0e-9;   // relative: a slider value this close to a rung counts as on it

bool onRung(double w, double rung) { return std::fabs(w - rung) <= kRungTol * rung; }
} // namespace

void Warp::dropStaleMemory()
{
    // Back at or above the remembered warp: there is nothing left to resume, a later slow-down starts a new memory
    if (hasSaved_ && value_ >= saved_) hasSaved_ = false;
}

void Warp::set(double w)
{
    if (std::isnan(w)) return;
    value_ = std::clamp(w, kWarpMin, kWarpMax);
    dropStaleMemory();
}

bool Warp::stepUp()
{
    for (double rung : kWarpLadder) {
        if (rung > value_ && !onRung(value_, rung)) {
            value_ = rung;
            dropStaleMemory();
            return true;
        }
    }
    return false;
}

bool Warp::stepDown()
{
    for (std::size_t i = kWarpLadder.size(); i-- > 0;) {
        const double rung = kWarpLadder[i];
        if (rung < value_ && !onRung(value_, rung)) {
            value_ = rung;
            return true;
        }
    }
    return false;
}

int Warp::ladderIndex() const
{
    for (std::size_t i = 0; i < kWarpLadder.size(); ++i)
        if (onRung(value_, kWarpLadder[i])) return static_cast<int>(i);
    return -1;
}

bool Warp::slowDownOneLevel()
{
    if (!(value_ > kWarpLadder.front()) || onRung(value_, kWarpLadder.front())) return false;
    // Remember where the slow-down started, once; a second step of the same slow-down keeps the first value.
    if (!canResume()) {
        saved_ = value_;
        hasSaved_ = true;
    }
    return stepDown();
}

bool Warp::resume()
{
    if (!hasSaved_) return false;
    const bool changed = saved_ != value_;
    value_ = saved_;
    hasSaved_ = false;
    return changed;
}

double Warp::sliderFromWarp(double warp)
{
    if (!(warp > 1.0)) return 0.0;
    return std::clamp(std::log10(warp) / 6.0, 0.0, 1.0);
}

double Warp::warpFromSlider(double position)
{
    if (std::isnan(position)) return 1.0;
    return std::pow(10.0, 6.0 * std::clamp(position, 0.0, 1.0));
}

} // namespace app
} // namespace cosmos
