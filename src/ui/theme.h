#pragma once
// The one place that decides how the UI looks (plan section 10: Universe Sandbox-style translucent dark panels).
// Everything is derived from the palette tokens below; widgets elsewhere ask the palette for a colour instead of
// hard-coding one.
#include "imgui.h"

namespace cosmos {
namespace ui {

// Nine palette tokens. Change them here and the whole UI follows.
struct Palette {
    ImVec4 panel   = ImVec4(0.055f, 0.063f, 0.082f, 0.86f);   // window background (translucent: the scene shows through)
    ImVec4 surface = ImVec4(0.125f, 0.141f, 0.176f, 0.90f);   // buttons, frames, plot backgrounds
    ImVec4 border  = ImVec4(0.235f, 0.267f, 0.329f, 0.70f);   // outlines, separators
    ImVec4 text    = ImVec4(0.905f, 0.925f, 0.955f, 1.00f);   // primary text
    ImVec4 textDim = ImVec4(0.560f, 0.600f, 0.670f, 1.00f);   // secondary text, disabled text
    ImVec4 accent  = ImVec4(0.255f, 0.640f, 0.980f, 1.00f);   // selection, sliders, active toolbar buttons
    ImVec4 good    = ImVec4(0.200f, 0.700f, 0.390f, 1.00f);   // Start, positive state
    ImVec4 warn    = ImVec4(0.930f, 0.690f, 0.240f, 1.00f);   // physics behind, resume warp
    ImVec4 danger  = ImVec4(0.900f, 0.320f, 0.300f, 1.00f);   // Stop, errors
};

// The palette in use (a single instance; dark is the default theme).
const Palette& palette();

// `a` brightened (factor > 1) or darkened (< 1), alpha kept.
ImVec4 shade(const ImVec4& c, float factor);
// `c` with its alpha replaced.
ImVec4 withAlpha(const ImVec4& c, float alpha);
// Packed colour for draw-list calls.
inline ImU32 u32(const ImVec4& c) { return ImGui::ColorConvertFloat4ToU32(c); }

// Applies the dark theme to the current ImGui context: colours from the palette, rounding, padding, and every pixel size
// multiplied by `dpiScale` (the font scale is set by the caller).
void applyTheme(float dpiScale);

} // namespace ui
} // namespace cosmos
