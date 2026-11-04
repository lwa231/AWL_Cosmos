#include "ui/bottom_bar.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include "IconsFontAwesome7.h"
#include "app/format.h"
#include "app/warp.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {

double nowSeconds()
{
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

// Tooltip that also shows on a disabled item.
void tip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
}

struct ButtonColours {
    ImVec4 base, hover, active;
};

ButtonColours tinted(const ImVec4& c)
{
    return {withAlpha(shade(c, 0.85f), 0.95f), shade(c, 1.05f), shade(c, 1.25f)};
}

// A button with an accent colour; `enabled` false greys it out and ignores clicks. Returns true when clicked.
bool coloured(const char* label, const ImVec4* tint, bool enabled, const char* tooltip, ImVec2 size = ImVec2(0.0f, 0.0f),
              const ImVec4* textColour = nullptr)
{
    if (textColour) ImGui::PushStyleColor(ImGuiCol_Text, *textColour);
    if (tint) {
        const ButtonColours c = tinted(*tint);
        ImGui::PushStyleColor(ImGuiCol_Button, c.base);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, c.hover);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, c.active);
    }
    if (!enabled) ImGui::BeginDisabled();
    const bool clicked = ImGui::Button(label, size);
    if (!enabled) ImGui::EndDisabled();
    if (tint) ImGui::PopStyleColor(3);
    if (textColour) ImGui::PopStyleColor();
    if (tooltip) tip(tooltip);
    return clicked && enabled;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// EventLog

void EventLog::push(const std::string& text)
{
    entries_.push_back({text, nowSeconds()});
    while (entries_.size() > kMaxEntries) entries_.pop_front();
}

double EventLog::secondsSinceLatest() const
{
    return entries_.empty() ? 1.0e9 : nowSeconds() - entries_.back().realTime;
}

// ---------------------------------------------------------------------------------------------------------------
// BottomBar

void BottomBar::noteWindowRect()
{
    const ImVec2 p = ImGui::GetWindowPos(), z = ImGui::GetWindowSize();
    occupied_.push_back(ImVec4(p.x, p.y, p.x + z.x, p.y + z.y));
}

void BottomBar::draw(RunHost& host, ImVec2 display, ImFont* emphasis)
{
    occupied_.clear();
    const Palette& pal = palette();
    const ImGuiStyle& st = ImGui::GetStyle();
    const float s = ImGui::GetFontSize() / 16.0f;
    const float frameH = ImGui::GetFrameHeight();
    const float lineH = ImGui::GetTextLineHeight();
    height_ = std::floor(st.WindowPadding.y * 2.0f + frameH + st.ItemSpacing.y + lineH + 2.0f * s);
    const float top = display.y - height_;
    const bool compact = display.x < 1180.0f * s;

    ImGui::SetNextWindowPos(ImVec2(0.0f, top));
    ImGui::SetNextWindowSize(ImVec2(display.x, height_));
    ImGui::SetNextWindowBgAlpha(0.90f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    const bool open = ImGui::Begin("##bottom_bar", nullptr, flags);
    ImGui::PopStyleVar(2);
    occupied_.push_back(ImVec4(0.0f, top, display.x, display.y));
    if (open) {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddLine(ImVec2(0.0f, top + 0.5f), ImVec2(display.x, top + 0.5f), u32(withAlpha(pal.accent, 0.35f)), 1.0f);

        const RunState rs = host.runState();
        const bool running = rs == RunState::Running;

        // ---- run controls ----
        if (coloured(ICON_FA_PLAY "  Start", &pal.good, host.canStart(),
                     rs == RunState::Stopped ? "Take a snapshot of the scene and run it" : "Continue running"))
            host.start();
        ImGui::SameLine();
        if (coloured(compact ? ICON_FA_PAUSE "##pause" : ICON_FA_PAUSE "  Pause", nullptr, host.canPause(), "Pause the clock (editing is allowed while paused)",
                     compact ? ImVec2(frameH * 1.3f, 0.0f) : ImVec2(0.0f, 0.0f)))
            host.pause();
        ImGui::SameLine();
        if (coloured(compact ? ICON_FA_STOP "##stop" : ICON_FA_STOP "  Stop", &pal.danger, host.canStop(),
                     "Stop and restore the scene as it was at Start", compact ? ImVec2(frameH * 1.3f, 0.0f) : ImVec2(0.0f, 0.0f)))
            host.stop();
        ImGui::SameLine();
        if (coloured(compact ? ICON_FA_FLOPPY_DISK "##save" : ICON_FA_FLOPPY_DISK "  Save as scene", nullptr, host.canSaveCurrent(),
                     "Save the current state, with mid-run edits, as a scene file",
                     compact ? ImVec2(frameH * 1.3f, 0.0f) : ImVec2(0.0f, 0.0f)))
            host.saveCurrentAsScene();

        // ---- time rate ----
        ImGui::SameLine(0.0f, 24.0f * s);
        const float sliderW = std::min(300.0f * s, std::max(150.0f * s, display.x * 0.19f));
        ImGui::SetNextItemWidth(sliderW);
        const double warp = host.warp();
        float pos = static_cast<float>(app::Warp::sliderFromWarp(warp));
        const std::string readout = app::formatTimeRate(warp);
        if (ImGui::SliderFloat("##warp", &pos, 0.0f, 1.0f, readout.c_str(), ImGuiSliderFlags_NoInput)) {
            // magnetic ladder rungs (1, 10, ... 1e6): a drag lands exactly on a rung when it is close to one
            const float rung = std::round(pos * 6.0f) / 6.0f;
            if (std::fabs(pos - rung) < 0.014f) pos = rung;
            host.setWarp(app::Warp::warpFromSlider(pos));
        }
        {   // rung ticks under the track
            const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
            const float inner = st.FramePadding.x + 0.5f * st.GrabMinSize;
            for (int k = 0; k < 7; ++k) {
                const float x = std::floor(a.x + inner + (b.x - a.x - 2.0f * inner) * static_cast<float>(k) / 6.0f) + 0.5f;
                dl->AddLine(ImVec2(x, b.y - 4.0f * s), ImVec2(x, b.y - 1.0f * s), u32(withAlpha(pal.textDim, 0.7f)), 1.0f);
            }
        }
        tip("Time rate: simulated time per real second (1 s/sec to 1e6 x)");

        if (host.canResumeWarp()) {
            ImGui::SameLine();
            const std::string tipText = "Back to " + app::formatTimeRate(host.resumeWarpValue()) + " after the automatic slow-down";
            const ImVec4 dark(0.10f, 0.08f, 0.02f, 1.0f);   // dark text reads better on the amber button
            if (coloured(ICON_FA_FORWARD_FAST "  Resume warp", &pal.warn, true, tipText.c_str(), ImVec2(0.0f, 0.0f), &dark)) host.resumeWarp();
        }

        // ---- date and clock ----
        ImGui::SameLine(0.0f, 24.0f * s);
        ImGui::AlignTextToFramePadding();
        if (emphasis) ImGui::PushFont(emphasis, 0.0f);
        ImGui::TextUnformatted(app::formatSimDate(host.displayTime()).c_str());
        if (emphasis) ImGui::PopFont();
        tip("Simulated date and clock");
        if (rs == RunState::Paused) {
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(pal.textDim, ICON_FA_PAUSE " paused");
        } else if (rs == RunState::Stopped) {
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(pal.textDim, "stopped");
        }
        if (running && host.physicsBehind()) {
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(pal.warn, ICON_FA_TRIANGLE_EXCLAMATION " physics behind");
            tip("The physics cannot keep up at this time rate: the clock waits for it");
        }

        // ---- toolbar (right aligned) ----
        const float iconW = frameH * 1.25f;
        const int buttons = 5;
        const float toolbarW = buttons * iconW + (buttons - 1) * st.ItemSpacing.x;
        const float wanted = display.x - st.WindowPadding.x - toolbarW;
        ImGui::SameLine(std::max(ImGui::GetItemRectMax().x + st.ItemSpacing.x, wanted));
        auto toolbarButton = [&](const char* icon, const char* id, const char* tooltip, bool& flag) {
            const bool on = flag;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, withAlpha(pal.accent, 0.60f));
            const std::string label = std::string(icon) + "##" + id;
            const bool clicked = ImGui::Button(label.c_str(), ImVec2(iconW, 0.0f));
            if (on) ImGui::PopStyleColor();
            tip(tooltip);
            if (clicked) flag = !flag;
            return clicked;
        };
        if (toolbarButton(ICON_FA_PEN_TO_SQUARE, "edit", "Edit: the scene list and object editor", toolbar.edit)) {}
        ImGui::SameLine();
        if (toolbarButton(ICON_FA_CIRCLE_PLUS, "add", "Add a body or a shuttle", toolbar.add)) {}
        ImGui::SameLine();
        if (toolbarButton(ICON_FA_EYE, "view", "View: names and overlays", toolbar.view) && toolbar.view) toolbar.sim = false;
        ImGui::SameLine();
        if (toolbarButton(ICON_FA_SLIDERS, "sim", "Simulation settings", toolbar.sim) && toolbar.sim) toolbar.view = false;
        ImGui::SameLine();
        toolbarButton(ICON_FA_CHART_LINE, "stats", "Stats: conservation drift and frame time", toolbar.stats);

        // ---- log line ----
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float w = ImGui::GetContentRegionAvail().x;
        ImGui::InvisibleButton("##log", ImVec2(w, lineH));
        const bool hovered = ImGui::IsItemHovered();
        if (ImGui::IsItemClicked()) showHistory_ = !showHistory_;
        const EventLog::Entry* last = log.latest();
        const bool fresh = last && log.secondsSinceLatest() < 8.0;
        std::string line = std::string(ICON_FA_MESSAGE "  ") + (last ? last->text : std::string("Ready"));
        const ImVec4 col = (fresh || hovered) ? pal.text : pal.textDim;
        dl->PushClipRect(p0, ImVec2(p0.x + w, p0.y + lineH), true);
        dl->AddText(p0, u32(col), line.c_str());
        dl->PopClipRect();
        if (hovered) ImGui::SetTooltip("Click to show the recent events");
    }
    ImGui::End();

    if (showHistory_) drawHistoryWindow(display, top);
    if (toolbar.view) drawViewWindow(display, top);
    if (toolbar.sim) drawSimWindow(host, display, top);
    if (toolbar.stats) {
        stats.draw(&toolbar.stats);
        float r[4];
        if (stats.windowRect(r)) occupied_.push_back(ImVec4(r[0], r[1], r[2], r[3]));
    }
}

void BottomBar::drawViewWindow(ImVec2 display, float barTop)
{
    const Palette& pal = palette();
    const float s = ImGui::GetFontSize() / 16.0f;
    ImGui::SetNextWindowPos(ImVec2(display.x - 12.0f * s, barTop - 10.0f * s), ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(260.0f * s, 0.0f), ImVec2(1.0e9f, 1.0e9f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("View", &toolbar.view, flags)) {
        ImGui::Checkbox("Names", &showLabels);
        ImGui::SeparatorText("Overlays");
        ImGui::BeginDisabled(!overlays.overlaysAvailable);
        ImGui::Checkbox("Velocity arrows", &overlays.velocityArrows);
        ImGui::Checkbox("Acceleration arrows", &overlays.accelerationArrows);
        ImGui::Checkbox("Trails", &overlays.trails);
        ImGui::Checkbox("Osculating orbits", &overlays.osculatingOrbits);
        ImGui::Checkbox("Predicted path", &overlays.predictedPath);
        ImGui::Checkbox("Gravity-well rings", &overlays.gravityRings);
        ImGui::Checkbox("Hill / SOI / Roche circles", &overlays.hillSoiRoche);
        ImGui::Checkbox("Atmosphere bubbles", &overlays.atmospheres);
        ImGui::EndDisabled();
        if (!overlays.overlaysAvailable) ImGui::TextColored(pal.textDim, "Overlays arrive in a later milestone.");
        noteWindowRect();
    }
    ImGui::End();
}

void BottomBar::drawSimWindow(RunHost& host, ImVec2 display, float barTop)
{
    const Palette& pal = palette();
    const float s = ImGui::GetFontSize() / 16.0f;
    ImGui::SetNextWindowPos(ImVec2(display.x - 12.0f * s, barTop - 10.0f * s), ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(320.0f * s, 0.0f), ImVec2(1.0e9f, 1.0e9f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("Simulation", &toolbar.sim, flags)) {
        ImGui::TextColored(pal.textDim, "Bodies: IAS15 (adaptive, 15th order)\nShuttles: DOPRI5 (adaptive Runge-Kutta)");
        ImGui::Spacing();

        int threads = host.threadCount();
        ImGui::SetNextItemWidth(160.0f * s);
        if (ImGui::SliderInt("Threads", &threads, 1, 16)) host.setThreadCount(threads);
        tip("Worker threads of the shuttle pass. Never changes a result.");

        const bool tickEditable = host.controllerTickEditable();
        double tick = host.controllerTick();
        ImGui::BeginDisabled(!tickEditable);
        ImGui::SetNextItemWidth(160.0f * s);
        if (ImGui::InputDouble("Controller tick (s)", &tick, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) host.setControllerTick(tick);
        ImGui::EndDisabled();
        tip(tickEditable ? "How often shuttle autopilots decide (simulated seconds). Press Enter to apply."
                         : "The controller tick can only be changed while the simulation is not running.");

        bool fuel = host.infiniteFuel();
        if (ImGui::Checkbox("Infinite fuel", &fuel)) host.setInfiniteFuel(fuel);
        tip("Shuttles never run out of fuel");

        bool anyShuttle = host.slowDownAnyShuttle();
        if (ImGui::Checkbox("Slow down for any shuttle impact", &anyShuttle)) host.setSlowDownAnyShuttle(anyShuttle);
        tip("Off: the warp steps down only for the followed shuttle and for bodies about to collide.\nOn: for every shuttle about to hit a body or its air.");
        noteWindowRect();
    }
    ImGui::End();
}

void BottomBar::drawHistoryWindow(ImVec2 display, float barTop)
{
    const Palette& pal = palette();
    const float s = ImGui::GetFontSize() / 16.0f;
    ImGui::SetNextWindowPos(ImVec2(12.0f * s, barTop - 10.0f * s), ImGuiCond_Always, ImVec2(0.0f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(std::min(display.x - 24.0f * s, 640.0f * s), 240.0f * s), ImGuiCond_Always);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("Recent events", &showHistory_, flags)) {
        if (log.empty()) ImGui::TextColored(pal.textDim, "Nothing has happened yet.");
        for (const EventLog::Entry& e : log.entries()) ImGui::TextUnformatted(e.text.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 2.0f) ImGui::SetScrollHereY(1.0f);   // stick to the newest line
        noteWindowRect();
    }
    ImGui::End();
}

} // namespace ui
} // namespace cosmos
