#pragma once
// Velocity and acceleration arrows (plan 9.5 first row): line glyphs, a shaft plus a two-segment head, in camera-relative
// metres. GL-free; the glyphs go into the overlay LineBatch (plan 9.4 step 9).
//
// Length: the slider is `lengthPx`, the on-screen length of an arrow whose magnitude equals the reference. The scaling
// mode shapes the response (x = magnitude / reference):
//   Linear  g = x
//   Sqrt    g = sqrt(x)
//   Log     g = 1 + log10(x) / kArrowLogDecades   (zero at six decades below the reference, 1 at the reference)
// and the length is clamped to kArrowMaxFactor * lengthPx so one fast object cannot run off the screen.
#include "app/render_frame.h"

namespace cosmos {
namespace app {

enum class ArrowScale { Linear, Sqrt, Log };

constexpr double kArrowLogDecades = 6.0;
constexpr double kArrowMaxFactor = 3.0;
constexpr double kArrowMinPx = 1.5;           // shorter arrows are not drawn (they would be a dot)

struct Rgba {
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;
};

// The shape function g(x) above, x >= 0 (negative or NaN gives 0).
double arrowShape(ArrowScale mode, double x);

// lengthPx * g(magnitude / reference), clamped to [0, kArrowMaxFactor * lengthPx]. 0 for a reference <= 0.
double arrowLengthPx(ArrowScale mode, double magnitude, double reference, double lengthPx);

// Appends shaft + head as three segments. (x, y): tail in camera-relative metres; (dirX, dirY): any non-zero direction
// (normalised here). Returns false (nothing added) when the length is under kArrowMinPx or the direction is zero.
bool appendArrow(LineBatch& batch, double x, double y, double dirX, double dirY, double lengthPx, double metresPerPixel,
                 const Rgba& colour, float widthPx = 1.5f);

} // namespace app
} // namespace cosmos
