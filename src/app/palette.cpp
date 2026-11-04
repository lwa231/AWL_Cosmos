#include "app/palette.h"
#include "core/rng.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

std::array<double, 3> oklchToLinearSrgbUnclipped(double L, double C, double hueDeg)
{
    const double h = hueDeg * 3.14159265358979323846 / 180.0;
    const double a = C * std::cos(h);
    const double b = C * std::sin(h);

    // OKLab -> LMS' (cube-root domain) -> LMS -> linear sRGB (Ottosson 2020)
    const double l_ = L + 0.3963377774 * a + 0.2158037573 * b;
    const double m_ = L - 0.1055613458 * a - 0.0638541728 * b;
    const double s_ = L - 0.0894841775 * a - 1.2914855480 * b;
    const double l = l_ * l_ * l_;
    const double m = m_ * m_ * m_;
    const double s = s_ * s_ * s_;

    return {+4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s};
}

double linearToSrgb(double x)
{
    x = std::min(1.0, std::max(0.0, x));
    return x <= 0.0031308 ? 12.92 * x : 1.055 * std::pow(x, 1.0 / 2.4) - 0.055;
}

double srgbToLinear(double x)
{
    x = std::min(1.0, std::max(0.0, x));
    return x <= 0.04045 ? x / 12.92 : std::pow((x + 0.055) / 1.055, 2.4);
}

namespace {
bool inGamut(const std::array<double, 3>& rgb)
{
    const double eps = 1e-6;   // tolerate rounding at the gamut boundary
    for (double c : rgb)
        if (c < -eps || c > 1.0 + eps) return false;
    return true;
}
} // namespace

SrgbColour oklchToSrgb(double L, double C, double hueDeg, double* chromaUsed)
{
    std::array<double, 3> lin = oklchToLinearSrgbUnclipped(L, C, hueDeg);
    if (!inGamut(lin)) {
        // Reduce chroma at constant lightness and hue. C = 0 (a grey) is always in gamut for 0 <= L <= 1.
        double lo = 0.0, hi = C;
        for (int i = 0; i < 40; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (inGamut(oklchToLinearSrgbUnclipped(L, mid, hueDeg))) lo = mid; else hi = mid;
        }
        C = lo;
        lin = oklchToLinearSrgbUnclipped(L, C, hueDeg);
    }
    if (chromaUsed) *chromaUsed = C;
    return {static_cast<float>(linearToSrgb(lin[0])), static_cast<float>(linearToSrgb(lin[1])),
            static_cast<float>(linearToSrgb(lin[2]))};
}

const std::array<SrgbColour, kPaletteHues>& paletteHues()
{
    static const std::array<SrgbColour, kPaletteHues> hues = [] {
        std::array<SrgbColour, kPaletteHues> h{};
        for (int k = 0; k < kPaletteHues; ++k)
            h[k] = oklchToSrgb(kPaletteLightness, kPaletteChroma, kPaletteHueOffsetDeg + 360.0 * k / kPaletteHues);
        return h;
    }();
    return hues;
}

HueTriple decodeTriple(std::size_t index)
{
    // a: 24 choices, b: 23 (all but a), c: 22 (all but a and b); lexicographic.
    const std::size_t perA = static_cast<std::size_t>(kPaletteHues - 1) * (kPaletteHues - 2);
    const std::size_t perB = kPaletteHues - 2;
    const std::size_t a = index / perA;
    std::size_t b = (index % perA) / perB;
    std::size_t c = index % perB;
    if (b >= a) ++b;                      // skip a
    const std::size_t lo = std::min(a, b), hi = std::max(a, b);
    if (c >= lo) ++c;                     // skip the smaller of a, b
    if (c >= hi) ++c;                     // then the larger
    HueTriple t;
    t.hue[0] = static_cast<std::uint8_t>(a);
    t.hue[1] = static_cast<std::uint8_t>(b);
    t.hue[2] = static_cast<std::uint8_t>(c);
    return t;
}

ShuttlePalette::ShuttlePalette(std::uint64_t seed) : seed_(seed), order_(kTripleCount)
{
    for (std::size_t i = 0; i < kTripleCount; ++i) order_[i] = static_cast<std::uint16_t>(i);
    // Fisher-Yates driven by the SplitMix64 stream of the seed.
    Rng rng(hashIdTick(seed, 0x7A11E77Eull));
    for (std::size_t i = kTripleCount - 1; i > 0; --i)
        std::swap(order_[i], order_[static_cast<std::size_t>(rng.below(i + 1))]);
}

std::array<SrgbColour, 3> ShuttlePalette::coloursFor(std::uint64_t shuttleId) const
{
    const HueTriple t = tripleFor(shuttleId);
    const auto& hues = paletteHues();
    return {hues[t.hue[0]], hues[t.hue[1]], hues[t.hue[2]]};
}

} // namespace app
} // namespace cosmos
