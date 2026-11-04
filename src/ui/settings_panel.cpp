#include "ui/settings_panel.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include "IconsFontAwesome7.h"
#include "app/format.h"
#include "app/units.h"
#include "ui/info_text.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {

float uiScale() { return ImGui::GetFontSize() / 16.0f; }

void tip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
}

void hint(const char* text, const ImVec4* colour = nullptr)
{
    ImGui::PushStyleColor(ImGuiCol_Text, colour ? *colour : palette().textDim);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}

// A label in a fixed column; the widget that follows is placed next to it.
void rowLabel(const char* label)
{
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine(210.0f * uiScale());
}

template <std::size_t N>
int indexOf(int value, const int (&choices)[N])
{
    for (std::size_t i = 0; i < N; ++i)
        if (choices[i] == value) return static_cast<int>(i);
    return 0;
}

bool qualityCombo(const char* id, app::Quality& q)
{
    static const char* names[] = {"Low", "Medium", "High"};
    int i = static_cast<int>(q);
    ImGui::SetNextItemWidth(180.0f * uiScale());
    if (!ImGui::Combo(id, &i, names, 3)) return false;
    q = static_cast<app::Quality>(std::clamp(i, 0, 2));
    return true;
}

} // namespace

void SettingsPanel::show(SettingsTab tab)
{
    open_ = true;
    pendingTab_ = static_cast<int>(tab);
}

bool SettingsPanel::drawGraphics(app::Settings& s, const SettingsPanelInfo& info)
{
    bool changed = false;
    const float k = uiScale();
    const Palette& pal = palette();

    rowLabel("Vertical sync");
    changed |= ImGui::Checkbox("##vsync", &s.vsync);
    tip("Waits for the display's refresh before showing a frame: no tearing, and the frame rate follows the display.");
    if (info.vsyncLocked) {
        ImGui::SameLine();
        hint("(--no-vsync is on for this run)");
    }

    rowLabel("Anti-aliasing (MSAA)");
    {
        static const char* names[] = {"Off", "2x", "4x", "8x"};
        int i = indexOf(s.msaa, app::kMsaaChoices);
        ImGui::SetNextItemWidth(180.0f * k);
        if (ImGui::Combo("##msaa", &i, names, 4)) {
            s.msaa = app::kMsaaChoices[std::clamp(i, 0, 3)];
            changed = true;
        }
        tip("Smooths the edges of bodies and shuttles. The window's framebuffer is created at launch, so a new value applies the next time the program starts.");
        if (s.msaa != info.launchMsaa) {
            ImGui::SameLine();
            ImGui::TextColored(pal.warn, ICON_FA_ROTATE_RIGHT " applies on next launch");
        }
    }

    rowLabel("UI scale");
    {
        bool automatic = s.uiScale <= 0.0f;
        if (ImGui::Checkbox("Automatic##uiscale", &automatic)) {
            s.uiScale = automatic ? 0.0f : 1.0f;
            changed = true;
        }
        tip("Automatic follows the display's content scale (Windows display scaling).");
        if (!automatic) {
            if (!scaleActive_) scaleEdit_ = s.uiScale;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(180.0f * k);
            ImGui::SliderFloat("##uiscaleval", &scaleEdit_, app::kMinUiScale, app::kMaxUiScale, "%.2fx", ImGuiSliderFlags_AlwaysClamp);
            scaleActive_ = ImGui::IsItemActive();
            if (ImGui::IsItemDeactivatedAfterEdit()) {   // applied on release: the window itself rescales while the slider is dragged
                s.uiScale = scaleEdit_;
                changed = true;
            }
            tip("Size of the text and panels. Applied when you let go of the slider. Ctrl+click to type a value.");
        } else {
            scaleActive_ = false;
        }
        if (info.uiScaleLocked) hint("--ui-scale is on for this run: the saved value is used the next time without it.");
    }

    rowLabel("Frame-rate cap");
    {
        static const char* names[] = {"Off", "60 fps", "120 fps", "144 fps", "240 fps"};
        int i = indexOf(s.fpsCap, app::kFpsCapChoices);
        ImGui::SetNextItemWidth(180.0f * k);
        if (ImGui::Combo("##fps", &i, names, 5)) {
            s.fpsCap = app::kFpsCapChoices[std::clamp(i, 0, 4)];
            changed = true;
        }
        tip("Limits how often a frame is drawn, to save power and heat. With vsync on, the display's refresh rate limits it too.");
    }

    ImGui::SeparatorText("Stars and sky");
    rowLabel("Star quality");
    changed |= qualityCombo("##starq", s.starQuality);
    tip("Detail of the procedural stars. Saved now; used by the visuals added in a later V1.1 step.");
    rowLabel("Skybox quality");
    changed |= qualityCombo("##skyq", s.skyboxQuality);
    tip("Detail of the procedural starfield and nebula haze. Saved now; used by a later V1.1 step.");
    return changed;
}

bool SettingsPanel::drawSimulation(app::Settings& s, const SettingsPanelInfo& info)
{
    bool changed = false;
    const float k = uiScale();

    hint("Defaults for every scene you open. The Simulation window in the toolbar changes the same things for the current run only.");
    ImGui::Spacing();

    rowLabel("Threads");
    {
        int n = s.threads;
        const std::string autoText = "Auto (" + std::to_string(std::max(1, std::min(app::kMaxThreadSetting, info.hardwareThreads))) + ")";
        ImGui::SetNextItemWidth(180.0f * k);
        if (ImGui::SliderInt("##threads", &n, 0, app::kMaxThreadSetting, n == 0 ? autoText.c_str() : "%d", ImGuiSliderFlags_AlwaysClamp)) {
            s.threads = n;
            changed = true;
        }
        tip("Worker threads of the shuttle pass. Auto uses the processor's thread count (at most 16). Never changes a result.");
    }

    rowLabel("Default warp");
    {
        double w = s.defaultWarp;
        const double lo = app::kMinDefaultWarp, hi = app::kMaxDefaultWarp;
        ImGui::SetNextItemWidth(180.0f * k);
        if (ImGui::SliderScalar("##warp", ImGuiDataType_Double, &w, &lo, &hi, "%.3g x", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp)) {
            s.defaultWarp = w;
            changed = true;
        }
        tip("Time rate (simulated seconds per real second) of a scene when you open it. Ctrl+click to type a value.");
        ImGui::SameLine();
        hint(app::formatTimeRate(s.defaultWarp).c_str());
    }

    rowLabel("Automatic slow-down");
    changed |= ImGui::Checkbox("##autoslow", &s.autoSlowDown);
    tip("Steps the time rate down as an impact approaches, so you can watch it. Off: the warp is never changed for you.");

    rowLabel("");
    ImGui::BeginDisabled(!s.autoSlowDown);
    changed |= ImGui::Checkbox("Slow down for any shuttle impact", &s.slowDownAnyShuttle);
    ImGui::EndDisabled();
    tip("Off: only the followed or selected shuttle and colliding bodies slow the warp down.\nOn: every shuttle about to hit a body or its air does.");

    rowLabel("Controller tick");
    {
        double t = s.controllerTick;
        ImGui::SetNextItemWidth(180.0f * k);
        if (ImGui::InputDouble("##tick", &t, 0.0, 0.0, "%.3f s", ImGuiInputTextFlags_EnterReturnsTrue)) {
            s.controllerTick = t;   // clamped by the caller
            changed = true;
        }
        tip("How often shuttle autopilots decide, in simulated seconds. Press Enter to apply. A scene file that sets its own tick keeps it.");
    }
    return changed;
}

bool SettingsPanel::drawDisplay(app::Settings& s)
{
    bool changed = false;
    const float k = uiScale();

    rowLabel("Length units");
    {
        static const char* names[] = {"Automatic (m, km, ... AU)", "Metres (SI)", "Kilometres", "Astronomical units (AU)"};
        int i = static_cast<int>(s.lengthUnit);
        ImGui::SetNextItemWidth(290.0f * k);
        if (ImGui::Combo("##len", &i, names, 4)) {
            s.lengthUnit = static_cast<app::LengthUnit>(std::clamp(i, 0, 3));
            changed = true;
        }
        tip("The unit of distances in the editor fields and readouts. Automatic picks the one that fits the size.");
    }
    rowLabel("Mass units");
    {
        static const char* names[] = {"Automatic (kg, Earth, Jupiter, Sun)", "Kilograms (SI)", "Earth masses", "Solar masses"};
        int i = static_cast<int>(s.massUnit);
        ImGui::SetNextItemWidth(290.0f * k);
        if (ImGui::Combo("##mass", &i, names, 4)) {
            s.massUnit = static_cast<app::MassUnit>(std::clamp(i, 0, 3));
            changed = true;
        }
        tip("The unit of body masses in the editor fields.");
    }
    rowLabel("Date format");
    {
        static const char* names[] = {"Day 123  04:05:06", "Day 123", "Year 2, Day 14  04:05:06"};
        int i = static_cast<int>(s.dateFormat);
        ImGui::SetNextItemWidth(290.0f * k);
        if (ImGui::Combo("##date", &i, names, 3)) {
            s.dateFormat = static_cast<app::DateFormat>(std::clamp(i, 0, 2));
            changed = true;
        }
        tip("How the simulated date is written in the bottom bar and the event log. Years are 365 days, counted from the scene's start.");
    }
    if (changed) app::applyDisplayPrefs(s);   // the preview below shows the new choice at once

    ImGui::Spacing();
    {
        const std::string preview = "Preview:  1 AU = " + app::formatDistance(149597870700.0) + "   Earth = " +
                                    app::formatQuantity(app::gmEarth(), app::Quantity::BodyMass, nullptr, 4) + "   " +
                                    app::formatSimDate(1.5e7);
        hint(preview.c_str());
    }

    ImGui::SeparatorText("Overlays on at startup");
    hint("What the View window shows when the program starts. Opening a scene later does not change the overlays.");
    struct Item { const char* label; bool app::StartupOverlays::*member; };
    static const Item items[] = {
        {"Names", &app::StartupOverlays::names},
        {"Velocity arrows", &app::StartupOverlays::velocityArrows},
        {"Acceleration arrows", &app::StartupOverlays::accelerationArrows},
        {"Trails", &app::StartupOverlays::trails},
        {"Osculating orbits", &app::StartupOverlays::osculatingOrbits},
        {"Predicted path", &app::StartupOverlays::predictedPath},
        {"Gravity-well rings", &app::StartupOverlays::gravityRings},
        {"Hill spheres", &app::StartupOverlays::hill},
        {"Spheres of influence", &app::StartupOverlays::soi},
        {"Roche limits", &app::StartupOverlays::roche},
        {"Atmosphere bubbles", &app::StartupOverlays::atmospheres},
    };
    if (ImGui::BeginTable("##ovtable", 2, ImGuiTableFlags_SizingStretchSame)) {
        for (const Item& it : items) {
            ImGui::TableNextColumn();
            changed |= ImGui::Checkbox(it.label, &(s.overlays.*(it.member)));
        }
        ImGui::EndTable();
    }
    return changed;
}

void SettingsPanel::drawControls()
{
    const Palette& pal = palette();
    const float k = uiScale();
    for (const BindingGroup& g : controlGroups()) {
        ImGui::SeparatorText(g.title);
        if (ImGui::BeginTable(g.title, 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoSavedSettings)) {
            ImGui::TableSetupColumn("keys", ImGuiTableColumnFlags_WidthFixed, 200.0f * k);
            ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthStretch);
            for (const Binding& b : g.bindings) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(pal.accent, "%s", b.keys);
                ImGui::TableNextColumn();
                ImGui::TextWrapped("%s", b.action);
            }
            ImGui::EndTable();
        }
    }
}

void SettingsPanel::drawCredits(ImFont* emphasis)
{
    const Palette& pal = palette();
    for (const CreditGroup& g : creditGroups()) {
        ImGui::SeparatorText(g.title);
        for (const CreditEntry& e : g.entries) {
            if (emphasis) ImGui::PushFont(emphasis, 0.0f);
            ImGui::TextUnformatted(e.name);
            if (emphasis) ImGui::PopFont();
            ImGui::SameLine();
            ImGui::TextColored(pal.textDim, "  %s", e.licence);
            ImGui::Indent(14.0f * uiScale());
            hint(e.what);
            ImGui::Unindent(14.0f * uiScale());
        }
        if (g.note) {
            ImGui::Spacing();
            hint(g.note);
        }
    }
    ImGui::SeparatorText("Development");
    ImGui::TextColored(pal.accent, "%s", developmentCredit());
    ImGui::Spacing();
    hint(licenceFileNote());
}

bool SettingsPanel::draw(app::Settings& settings, ImVec2 display, const SettingsPanelInfo& info, ImFont* emphasis)
{
    if (!open_) return false;
    bool changed = false;
    const float k = uiScale();
    const Palette& pal = palette();

    const ImVec2 size(std::min(display.x - 32.0f, 700.0f * k), std::min(display.y - 32.0f, 680.0f * k));
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, 0.5f * display.y), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    if (pendingTab_ >= 0) ImGui::SetNextWindowFocus();
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, withAlpha(pal.panel, 1.0f));
    const bool visible = ImGui::Begin("Settings", &open_, flags);
    ImGui::PopStyleColor();
    if (visible) {
        const float footer = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y + 4.0f * k;
        if (ImGui::BeginTabBar("##settingstabs")) {
            struct Tab { const char* label; SettingsTab id; };
            static const Tab tabs[] = {{ICON_FA_DISPLAY "  Graphics", SettingsTab::Graphics},
                                       {ICON_FA_SLIDERS "  Simulation", SettingsTab::Simulation},
                                       {ICON_FA_RULER "  Display & units", SettingsTab::Display},
                                       {ICON_FA_KEYBOARD "  Controls", SettingsTab::Controls},
                                       {ICON_FA_HEART "  Credits", SettingsTab::Credits}};
            for (const Tab& t : tabs) {
                const bool select = pendingTab_ == static_cast<int>(t.id);
                if (ImGui::BeginTabItem(t.label, nullptr, select ? ImGuiTabItemFlags_SetSelected : 0)) {
                    ImGui::BeginChild("##tabbody", ImVec2(0.0f, -footer), ImGuiChildFlags_None);
                    ImGui::Spacing();
                    switch (t.id) {
                    case SettingsTab::Graphics: changed |= drawGraphics(settings, info); break;
                    case SettingsTab::Simulation: changed |= drawSimulation(settings, info); break;
                    case SettingsTab::Display: changed |= drawDisplay(settings); break;
                    case SettingsTab::Controls: drawControls(); break;
                    case SettingsTab::Credits: drawCredits(emphasis); break;
                    }
                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        pendingTab_ = -1;

        ImGui::Separator();
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(pal.textDim, ICON_FA_FLOPPY_DISK "  Saved to %s", info.settingsFile);
        const ImGuiStyle& st = ImGui::GetStyle();
        const float restoreW = ImGui::CalcTextSize(ICON_FA_ARROW_ROTATE_LEFT "  Restore defaults").x + 2.0f * st.FramePadding.x;
        const float closeW = ImGui::CalcTextSize(ICON_FA_XMARK "  Close").x + 2.0f * st.FramePadding.x;
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - restoreW - closeW - st.ItemSpacing.x);
        if (ImGui::Button(ICON_FA_ARROW_ROTATE_LEFT "  Restore defaults")) {
            settings = app::Settings();
            app::applyDisplayPrefs(settings);
            changed = true;
        }
        tip("Puts every setting back to its default");
        ImGui::SameLine();
        if (ImGui::Button(ICON_FA_XMARK "  Close")) open_ = false;
    }
    ImGui::End();
    if (changed) app::clampSettings(settings);
    return changed;
}

} // namespace ui
} // namespace cosmos
