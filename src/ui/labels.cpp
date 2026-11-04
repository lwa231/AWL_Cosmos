#include "ui/labels.h"
#include <algorithm>
#include <numeric>
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {

struct Rect {
    float x0, y0, x1, y1;
};

bool overlap(const Rect& a, const Rect& b, float pad)
{
    return a.x0 < b.x1 + pad && b.x0 < a.x1 + pad && a.y0 < b.y1 + pad && b.y0 < a.y1 + pad;
}

} // namespace

std::vector<PlacedLabel> layoutLabels(const std::vector<LabelItem>& items, const LabelOptions& options, ImVec2 screen,
                                      const std::function<ImVec2(const LabelItem&)>& measure)
{
    std::vector<PlacedLabel> placed;
    if (!options.show || options.maxLabels <= 0) return placed;

    // Priority order; ties keep input order (stable) so the picture does not flicker between frames.
    std::vector<std::size_t> order(items.size());
    std::iota(order.begin(), order.end(), std::size_t(0));
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (items[a].highlighted != items[b].highlighted) return items[a].highlighted;
        return items[a].priority > items[b].priority;
    });

    std::vector<Rect> taken;
    for (const ImVec4& k : options.keepOut) taken.push_back({k.x, k.y, k.z, k.w});
    int shuttles = 0;

    for (std::size_t idx : order) {
        if (static_cast<int>(placed.size()) >= options.maxLabels) break;
        const LabelItem& it = items[idx];
        if (it.text.empty()) continue;
        if (it.shuttle && !it.highlighted && shuttles >= options.maxShuttleLabels) continue;
        const ImVec2 size = measure(it);
        const float off = it.radiusPx * 0.7071f + options.gapPx;   // diagonal offset from the marker edge
        const ImVec2 tl(it.pos.x + off, it.pos.y - off - size.y);
        const Rect r{tl.x, tl.y, tl.x + size.x, tl.y + size.y};
        if (r.x0 < 0.0f || r.y0 < 0.0f || r.x1 > screen.x || r.y1 > screen.y) continue;
        // The marker itself must be on screen too (a label hanging in from beyond the edge looks detached).
        if (it.pos.x < -it.radiusPx || it.pos.y < -it.radiusPx || it.pos.x > screen.x + it.radiusPx || it.pos.y > screen.y + it.radiusPx)
            continue;
        bool free = true;
        for (const Rect& t : taken)
            if (overlap(r, t, options.padPx)) { free = false; break; }
        if (!free) continue;
        taken.push_back(r);
        placed.push_back({idx, tl, size});
        if (it.shuttle) ++shuttles;
    }
    return placed;
}

int drawLabels(const std::vector<LabelItem>& items, const LabelOptions& optionsIn, float uiScale)
{
    if (!optionsIn.show || items.empty()) return 0;
    LabelOptions options = optionsIn;
    options.gapPx *= uiScale;
    options.padPx *= uiScale;

    ImGuiStyle& style = ImGui::GetStyle();
    const float baseSize = style.FontSizeBase;
    const float shuttleSize = baseSize * 0.85f;
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    auto measure = [&](const LabelItem& it) {
        ImGui::PushFont(nullptr, it.shuttle ? shuttleSize : baseSize);
        const ImVec2 size = ImGui::CalcTextSize(it.text.c_str());
        ImGui::PopFont();
        return size;
    };
    const std::vector<PlacedLabel> placed = layoutLabels(items, options, screen, measure);

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    for (const PlacedLabel& p : placed) {
        const LabelItem& it = items[p.item];
        ImVec4 col = it.colour;
        col.w = it.highlighted ? 1.0f : (it.shuttle ? 0.72f : 0.90f);
        if (it.highlighted) col = palette().text;
        ImGui::PushFont(nullptr, it.shuttle ? shuttleSize : baseSize);
        ImFont* font = ImGui::GetFont();
        const float size = ImGui::GetFontSize();
        const float shadow = std::max(1.0f, uiScale);
        // soft dark outline so the text reads over bright discs and over the overlay lines
        dl->AddText(font, size, ImVec2(p.topLeft.x + shadow, p.topLeft.y + shadow), u32(ImVec4(0.0f, 0.0f, 0.0f, 0.85f * col.w)), it.text.c_str());
        dl->AddText(font, size, p.topLeft, u32(col), it.text.c_str());
        ImGui::PopFont();
    }
    return static_cast<int>(placed.size());
}

} // namespace ui
} // namespace cosmos
