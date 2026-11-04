#pragma once
// Body and shuttle name labels (plan 9.4 step 10: drawn into ImGui's foreground draw list, above the scene and below the
// ImGui windows' own content order, i.e. on top of everything the renderer drew).
//
// The app turns each visible object into a LabelItem (position of its marker in ImGui screen coordinates, drawn radius in
// pixels, name, a priority). layoutLabels() decides which labels are shown and where:
//   - a label sits up and to the right of its marker, offset by the marker radius plus a gap;
//   - items are tried in priority order (highest first); a label that would overlap one already placed, would leave the
//     screen or would fall inside a keep-out rectangle (the bottom bar, side panels) is dropped;
//   - at most LabelOptions::maxLabels labels are shown.
// layoutLabels() is pure (it measures text through a callback), drawLabels() draws the result.
#include <functional>
#include <string>
#include <vector>
#include "imgui.h"

namespace cosmos {
namespace ui {

struct LabelItem {
    ImVec2 pos;                   // marker centre, ImGui screen coordinates (pixels, y down)
    float radiusPx = 3.0f;        // drawn marker radius
    std::string text;
    ImVec4 colour = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    float priority = 0.0f;        // higher wins a contested spot
    bool shuttle = false;         // shuttle labels are smaller and dimmer
    bool highlighted = false;     // the followed object: bright, always tried first
};

struct LabelOptions {
    bool show = true;
    int maxLabels = 40;
    int maxShuttleLabels = 12;    // shuttles are many and small; this many at most
    float gapPx = 5.0f;           // marker edge to label
    float padPx = 3.0f;           // clearance kept between two labels
    std::vector<ImVec4> keepOut;  // rectangles (x0, y0, x1, y1) where no label is drawn
};

struct PlacedLabel {
    std::size_t item = 0;         // index into the items passed to layoutLabels
    ImVec2 topLeft;               // top-left corner of the text
    ImVec2 size;                  // text size
};

// Which labels are shown and where. `screen` is the display size; `measure` returns the size of a text string (for a
// shuttle label too). Result is in priority order.
std::vector<PlacedLabel> layoutLabels(const std::vector<LabelItem>& items, const LabelOptions& options, ImVec2 screen,
                                      const std::function<ImVec2(const LabelItem&)>& measure);

// Draws the labels of `items` into the foreground draw list (call between NewFrame and Render). `uiScale` scales the
// pixel distances of `options`. Returns the number drawn.
int drawLabels(const std::vector<LabelItem>& items, const LabelOptions& options, float uiScale);

} // namespace ui
} // namespace cosmos
