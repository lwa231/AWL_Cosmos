#include "app/warp.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "app/format.h"

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
    if (warp >= kWarpTopStop) return 1.0;
    return std::clamp(std::log(warp) / std::log(kWarpTopStop), 0.0, 1.0);
}

double Warp::warpFromSlider(double position)
{
    if (std::isnan(position)) return 1.0;
    const double p = std::clamp(position, 0.0, 1.0);
    if (p >= 1.0) return kWarpTopStop;
    return std::pow(kWarpTopStop, p);
}

double Warp::warpFromSliderDrag(double position, bool smooth)
{
    const double w = warpFromSlider(position);
    return smooth ? w : nearestStop(w);
}

double nearestStop(double warp)
{
    if (!(warp > kWarpLadder.front())) return kWarpLadder.front();   // NaN too
    for (std::size_t i = 0; i + 1 < kWarpLadder.size(); ++i) {
        const double lo = kWarpLadder[i], hi = kWarpLadder[i + 1];
        if (warp <= hi) return warp * warp < lo * hi ? lo : hi;   // compare with the geometric midpoint sqrt(lo hi)
    }
    return kWarpLadder.back();
}

std::string formatWarp(double warp)
{
    if (!std::isfinite(warp)) return "-- /s";
    const double a = std::fabs(warp);
    const char* unit = "s";
    double v = warp;
    // The unit switches where the readout would otherwise show 60 s, 60 min or 24 h
    if (a >= 86400.0 * (1.0 - 1.0e-9)) { unit = "d"; v = warp / 86400.0; }
    else if (a >= 3600.0 * (1.0 - 1.0e-9)) { unit = "h"; v = warp / 3600.0; }
    else if (a >= 60.0 * (1.0 - 1.0e-9)) { unit = "min"; v = warp / 60.0; }
    std::string num;
    const double r = std::round(v);
    if (std::fabs(v - r) <= 1.0e-9 * std::max(1.0, std::fabs(v))) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.0f", r);
        num = buf;
    } else {
        num = formatSig(v, 3);
        if (num.find('.') != std::string::npos) {   // "1.50" -> "1.5", "2.00" -> "2"
            while (!num.empty() && num.back() == '0') num.pop_back();
            if (!num.empty() && num.back() == '.') num.pop_back();
        }
    }
    if (num == "-0") num = "0";
    return num + " " + unit + "/s";
}

} // namespace app
} // namespace cosmos
