#pragma once
// Shuttle colour palette (plan 7.4): 24 hues spread evenly in OKLCH at one lightness and chroma, readable on black,
// and a seeded shuffle of every ordered hue triple so shuttles get unique colour triples in ID order.
// Pure logic, no GL. Colours are sRGB-encoded in [0,1].
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cosmos {
namespace app {

struct SrgbColour {
    float r = 0.0f, g = 0.0f, b = 0.0f;
};

// OKLCH (Ottosson's OKLab in polar form; hue in degrees) -> linear sRGB WITHOUT any clipping. Components outside
// [0,1] mean the colour is outside the sRGB gamut.
std::array<double, 3> oklchToLinearSrgbUnclipped(double L, double C, double hueDeg);

// Linear <-> sRGB transfer function (IEC 61966-2-1), input and output in [0,1] (input is clamped).
double linearToSrgb(double linear);
double srgbToLinear(double srgb);

// OKLCH -> sRGB (gamma encoded). If the colour is outside the gamut the chroma is reduced (binary search at constant
// L and h) until it fits; *chromaUsed (optional) reports the chroma finally used.
SrgbColour oklchToSrgb(double L, double C, double hueDeg, double* chromaUsed = nullptr);

constexpr int kPaletteHues = 24;
constexpr double kPaletteLightness = 0.76;   // OKLab L: bright enough on black, dark enough to keep chroma
constexpr double kPaletteChroma = 0.15;      // OKLab C requested for every hue (reduced only where the gamut forces it)
constexpr double kPaletteHueOffsetDeg = 20.0;

// The 24 palette colours, hue k at kPaletteHueOffsetDeg + 15 k degrees.
const std::array<SrgbColour, kPaletteHues>& paletteHues();

struct HueTriple {
    std::uint8_t hue[3] = {0, 0, 0};   // indices into paletteHues(), all three different
};

// Number of ordered triples of three different hues: 24 * 23 * 22 = 12144.
constexpr std::size_t kTripleCount = static_cast<std::size_t>(kPaletteHues) * (kPaletteHues - 1) * (kPaletteHues - 2);

// Decode triple number 0..kTripleCount-1 in lexicographic order. Every triple has exactly one number.
HueTriple decodeTriple(std::size_t index);

// A seeded deterministic shuffle of all kTripleCount ordered triples. The same seed always gives the same order on
// every machine (it uses cosmos::Rng, SplitMix64, and integer arithmetic only).
class ShuttlePalette {
public:
    explicit ShuttlePalette(std::uint64_t seed = 0);

    // The triple for a shuttle, by shuttle ID (plan 4.3 IDs are permanent and issued in creation order). IDs
    // that differ by less than kTripleCount never share a triple; after that the sequence repeats.
    HueTriple tripleFor(std::uint64_t shuttleId) const { return decodeTriple(order_[shuttleId % kTripleCount]); }

    // The three sector colours for a shuttle.
    std::array<SrgbColour, 3> coloursFor(std::uint64_t shuttleId) const;

    std::uint64_t seed() const { return seed_; }

private:
    std::uint64_t seed_;
    std::vector<std::uint16_t> order_;   // permutation of 0..kTripleCount-1
};

} // namespace app
} // namespace cosmos
