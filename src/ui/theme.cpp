#include "ui/theme.h"
#include <algorithm>

namespace cosmos {
namespace ui {

const Palette& palette()
{
    static const Palette p;
    return p;
}

ImVec4 shade(const ImVec4& c, float f)
{
    auto k = [f](float v) { return std::min(1.0f, std::max(0.0f, v * f)); };
    return ImVec4(k(c.x), k(c.y), k(c.z), c.w);
}

ImVec4 withAlpha(const ImVec4& c, float alpha) { return ImVec4(c.x, c.y, c.z, alpha); }

void applyTheme(float dpiScale)
{
    const Palette& p = palette();
    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();   // known starting point (also the unscaled sizes ScaleAllSizes works from)

    s.WindowPadding = ImVec2(12.0f, 8.0f);
    s.FramePadding = ImVec2(10.0f, 5.0f);
    s.ItemSpacing = ImVec2(8.0f, 6.0f);
    s.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    s.ScrollbarSize = 12.0f;
    s.GrabMinSize = 14.0f;
    s.WindowBorderSize = 1.0f;
    s.ChildBorderSize = 1.0f;
    s.PopupBorderSize = 1.0f;
    s.FrameBorderSize = 0.0f;
    s.TabBorderSize = 0.0f;
    s.WindowRounding = 8.0f;
    s.ChildRounding = 6.0f;
    s.FrameRounding = 6.0f;
    s.PopupRounding = 8.0f;
    s.ScrollbarRounding = 6.0f;
    s.GrabRounding = 6.0f;
    s.TabRounding = 6.0f;
    s.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    s.ButtonTextAlign = ImVec2(0.5f, 0.5f);
    s.SeparatorTextBorderSize = 1.0f;
    s.ScaleAllSizes(dpiScale);

    ImVec4* c = s.Colors;
    const ImVec4 clear(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_Text] = p.text;
    c[ImGuiCol_TextDisabled] = p.textDim;
    c[ImGuiCol_WindowBg] = p.panel;
    c[ImGuiCol_ChildBg] = clear;
    c[ImGuiCol_PopupBg] = withAlpha(shade(p.panel, 1.15f), 0.96f);
    c[ImGuiCol_Border] = p.border;
    c[ImGuiCol_BorderShadow] = clear;
    c[ImGuiCol_FrameBg] = p.surface;
    c[ImGuiCol_FrameBgHovered] = shade(p.surface, 1.35f);
    c[ImGuiCol_FrameBgActive] = shade(p.surface, 1.60f);
    c[ImGuiCol_TitleBg] = withAlpha(p.panel, 0.95f);
    c[ImGuiCol_TitleBgActive] = withAlpha(shade(p.panel, 1.25f), 0.98f);
    c[ImGuiCol_TitleBgCollapsed] = withAlpha(p.panel, 0.70f);
    c[ImGuiCol_MenuBarBg] = p.surface;
    c[ImGuiCol_ScrollbarBg] = clear;
    c[ImGuiCol_ScrollbarGrab] = p.border;
    c[ImGuiCol_ScrollbarGrabHovered] = shade(p.border, 1.4f);
    c[ImGuiCol_ScrollbarGrabActive] = p.accent;
    c[ImGuiCol_CheckMark] = p.accent;
    c[ImGuiCol_SliderGrab] = p.accent;
    c[ImGuiCol_SliderGrabActive] = shade(p.accent, 1.2f);
    c[ImGuiCol_Button] = p.surface;
    c[ImGuiCol_ButtonHovered] = shade(p.surface, 1.45f);
    c[ImGuiCol_ButtonActive] = withAlpha(p.accent, 0.80f);
    c[ImGuiCol_Header] = withAlpha(p.accent, 0.28f);
    c[ImGuiCol_HeaderHovered] = withAlpha(p.accent, 0.45f);
    c[ImGuiCol_HeaderActive] = withAlpha(p.accent, 0.65f);
    c[ImGuiCol_Separator] = p.border;
    c[ImGuiCol_SeparatorHovered] = p.accent;
    c[ImGuiCol_SeparatorActive] = shade(p.accent, 1.2f);
    c[ImGuiCol_ResizeGrip] = withAlpha(p.border, 0.5f);
    c[ImGuiCol_ResizeGripHovered] = withAlpha(p.accent, 0.7f);
    c[ImGuiCol_ResizeGripActive] = p.accent;
    c[ImGuiCol_Tab] = p.surface;
    c[ImGuiCol_TabHovered] = withAlpha(p.accent, 0.6f);
    c[ImGuiCol_TabSelected] = withAlpha(p.accent, 0.45f);
    c[ImGuiCol_PlotLines] = p.accent;
    c[ImGuiCol_PlotLinesHovered] = p.warn;
    c[ImGuiCol_PlotHistogram] = p.accent;
    c[ImGuiCol_PlotHistogramHovered] = p.warn;
    c[ImGuiCol_TableHeaderBg] = p.surface;
    c[ImGuiCol_TableBorderStrong] = p.border;
    c[ImGuiCol_TableBorderLight] = withAlpha(p.border, 0.5f);
    c[ImGuiCol_TableRowBg] = clear;
    c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.03f);
    c[ImGuiCol_TextSelectedBg] = withAlpha(p.accent, 0.40f);
    c[ImGuiCol_DragDropTarget] = p.warn;
    c[ImGuiCol_NavCursor] = p.accent;
    c[ImGuiCol_NavWindowingHighlight] = withAlpha(p.text, 0.7f);
    c[ImGuiCol_NavWindowingDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.45f);
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.55f);
}

} // namespace ui
} // namespace cosmos
