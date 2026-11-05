#include "ui/main_menu.h"
#include <algorithm>
#include <cfloat>
#include "IconsFontAwesome7.h"
#include "app/presets.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {

float uiScale() { return ImGui::GetFontSize() / 16.0f; }

const char* iconFor(const std::string& presetId)
{
    if (presetId == "lucerna") return ICON_FA_GLOBE;
    if (presetId == "solar_system") return ICON_FA_SUN;
    if (presetId == "empty") return ICON_FA_FILE;
    return ICON_FA_ATOM;
}

// A button of the menus: `primary` is drawn in the accent colour.
bool menuButton(const char* label, ImVec2 size, bool primary = false, bool danger = false)
{
    const Palette& pal = palette();
    int pushed = 0;
    if (primary) {
        ImGui::PushStyleColor(ImGuiCol_Button, withAlpha(pal.accent, 0.55f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, withAlpha(pal.accent, 0.80f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, pal.accent);
        pushed = 3;
    } else if (danger) {
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, withAlpha(pal.danger, 0.60f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, withAlpha(pal.danger, 0.85f));
        pushed = 2;
    }
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(pushed);
    return clicked;
}

// Draws `text` centred on the window's horizontal middle line, in `font` at `size` (unscaled pixels; 0 = current size).
void centredText(const char* text, ImFont* font, float size, const ImVec4& colour)
{
    if (font) ImGui::PushFont(font, size);
    const float w = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX(std::max(0.0f, 0.5f * (ImGui::GetWindowWidth() - w)));
    ImGui::TextColored(colour, "%s", text);
    if (font) ImGui::PopFont();
}

} // namespace

void MainMenu::showStart()
{
    mode_ = MenuMode::Start;
    message_.clear();
    openLoadPopup_ = false;
}

void MainMenu::showPause(const std::string& sceneName, bool wasRunning)
{
    mode_ = MenuMode::Pause;
    sceneName_ = sceneName;
    wasRunning_ = wasRunning;
}

void MainMenu::hide()
{
    mode_ = MenuMode::Hidden;
    openLoadPopup_ = false;
}

void MainMenu::setMessage(const std::string& text, bool isError)
{
    message_ = text;
    messageIsError_ = isError;
}

void MainMenu::askDiscard(MenuAction then, const std::string& savePath)
{
    confirmThen_ = then;
    confirmOpen_ = false;
    confirmSavePath_ = savePath;
}

MenuRequest MainMenu::drawConfirm(ImVec2 display, ImFont* titleFont)
{
    MenuRequest req;
    if (confirmThen_ == MenuAction::None) return req;
    const Palette& pal = palette();
    const float k = uiScale();
    const char* id = "Unsaved changes##discard";
    if (!confirmOpen_) {
        ImGui::OpenPopup(id);
        confirmOpen_ = true;
    }
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, 0.5f * display.y), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22.0f * k, 18.0f * k));
    const bool open = ImGui::BeginPopupModal(id, nullptr,
                                             ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                                                 ImGuiWindowFlags_NoSavedSettings);
    ImGui::PopStyleVar();
    if (!open) {   // closed from outside (it should not be): forget the question
        confirmThen_ = MenuAction::None;
        return req;
    }
    if (titleFont) ImGui::PushFont(titleFont, 24.0f);
    ImGui::TextColored(pal.text, "Discard unsaved changes?");
    if (titleFont) ImGui::PopFont();
    ImGui::Spacing();
    ImGui::TextColored(pal.textDim, "%s", confirmThen_ == MenuAction::Quit ? "The scene has edits that were not saved. Quit anyway?"
                                                                             : "The scene has edits that were not saved. Return to the menu anyway?");
    ImGui::Spacing();
    ImGui::Spacing();
    const ImVec2 btn(130.0f * k, 36.0f * k);
    MenuAction then = confirmThen_;
    bool close = false;
    if (menuButton(ICON_FA_FLOPPY_DISK "  Save...", btn, true)) {
        req.action = then;
        req.confirmed = true;
        req.saveFirst = true;
        close = true;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip)) ImGui::SetTooltip("Save the scene as %s, then continue", confirmSavePath_.c_str());
    ImGui::SameLine();
    if (menuButton(ICON_FA_TRASH "  Discard", btn, false, true)) {
        req.action = then;
        req.confirmed = true;
        close = true;
    }
    ImGui::SameLine();
    if (menuButton(ICON_FA_XMARK "  Cancel", btn)) close = true;
    if (close) {
        confirmThen_ = MenuAction::None;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
    return req;
}

MenuRequest MainMenu::draw(ImVec2 display, ImFont* titleFont, bool blocked)
{
    switch (mode_) {
    case MenuMode::Start: return drawStart(display, titleFont, blocked);
    case MenuMode::Pause: return drawPause(display, titleFont, blocked);
    case MenuMode::Hidden: break;
    }
    return {};
}

MenuRequest MainMenu::drawStart(ImVec2 display, ImFont* titleFont, bool blocked)
{
    MenuRequest req;
    const Palette& pal = palette();
    const float k = uiScale();

    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(display);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));   // the renderer's starless background shows through
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    const bool visible = ImGui::Begin("##startscreen", nullptr, flags);
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor();
    if (visible) {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        // A soft glow behind the cards: a vertical gradient from the background to a slightly bluer tone at the bottom
        dl->AddRectFilledMultiColor(ImVec2(0.0f, 0.0f), display, IM_COL32(0, 0, 0, 0), IM_COL32(0, 0, 0, 0), IM_COL32(26, 38, 70, 150),
                                    IM_COL32(26, 38, 70, 150));

        ImGui::BeginDisabled(blocked);

        const float contentW = std::min(display.x - 48.0f * k, 920.0f * k);
        const float left = 0.5f * (display.x - contentW);
        const float blockH = 500.0f * k;
        float y = std::max(20.0f * k, 0.5f * (display.y - blockH));

        ImGui::SetCursorPosY(y);
        centredText("AWL Cosmos", titleFont, 60.0f, pal.text);
        centredText("Orbital sandbox", nullptr, 0.0f, pal.textDim);
        y = ImGui::GetCursorPosY() + 36.0f * k;

        // ---- scene cards: files first, then the built-in ones (Lucerna, Real solar system, Empty sandbox) ----
        std::vector<const app::PresetInfo*> cards;
        for (const app::PresetInfo& p : app::presets()) cards.push_back(&p);
        std::stable_partition(cards.begin(), cards.end(), [](const app::PresetInfo* p) { return !p->file.empty(); });
        const int n = static_cast<int>(cards.size());
        const float gap = 18.0f * k;
        const float cardW = n > 0 ? (contentW - gap * static_cast<float>(n - 1)) / static_cast<float>(n) : contentW;
        const float cardH = 196.0f * k;
        const float pad = 18.0f * k;
        const ImVec2 origin = ImGui::GetWindowPos();
        for (int i = 0; i < n; ++i) {
            const app::PresetInfo& p = *cards[static_cast<std::size_t>(i)];
            const bool available = app::presetAvailable(p, app::defaultSceneDir());
            ImGui::SetCursorPos(ImVec2(left + static_cast<float>(i) * (cardW + gap), y));
            const ImVec2 p0 = ImGui::GetCursorScreenPos();
            const ImVec2 p1(p0.x + cardW, p0.y + cardH);
            ImGui::PushID(p.id.c_str());
            ImGui::BeginDisabled(!available);
            const bool clicked = ImGui::InvisibleButton("##card", ImVec2(cardW, cardH));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !blocked;
            ImGui::EndDisabled();
            ImVec4 fill = withAlpha(pal.surface, 0.80f);
            if (hovered && available) fill = withAlpha(shade(pal.surface, 1.45f), 0.92f);
            dl->AddRectFilled(p0, p1, u32(fill), 12.0f * k);
            dl->AddRect(p0, p1, u32(hovered && available ? pal.accent : pal.border), 12.0f * k, 0, hovered && available ? 2.0f : 1.0f);

            const ImVec4 textCol = available ? pal.text : pal.textDim;
            ImGui::SetCursorScreenPos(ImVec2(p0.x + pad, p0.y + pad));
            if (titleFont) ImGui::PushFont(titleFont, 34.0f);
            ImGui::TextColored(available ? pal.accent : pal.textDim, "%s", iconFor(p.id));
            if (titleFont) ImGui::PopFont();
            ImGui::SetCursorScreenPos(ImVec2(p0.x + pad, p0.y + pad + 52.0f * k));
            if (titleFont) ImGui::PushFont(titleFont, 22.0f);
            ImGui::TextColored(textCol, "%s", p.name.c_str());
            if (titleFont) ImGui::PopFont();
            ImGui::SetCursorScreenPos(ImVec2(p0.x + pad, p0.y + pad + 86.0f * k));
            ImGui::PushTextWrapPos(p1.x - pad);
            ImGui::TextColored(pal.textDim, "%s", available ? p.description.c_str() : "The scene file of this preset is missing.");
            ImGui::PopTextWrapPos();
            ImGui::PopID();
            if (clicked && available && !blocked) {
                req.action = MenuAction::NewPreset;
                req.arg = p.id;
            }
        }
        (void)origin;
        y += cardH + 30.0f * k;

        // ---- buttons ----
        const float btnW = 200.0f * k, btnH = 44.0f * k, btnGap = 14.0f * k;
        const float rowW = 3.0f * btnW + 2.0f * btnGap;
        ImGui::SetCursorPos(ImVec2(0.5f * (display.x - rowW), y));
        if (menuButton(ICON_FA_FOLDER_OPEN "  Load scene file...", ImVec2(btnW, btnH))) {
            files_ = app::listSceneFiles(app::defaultSceneDir());
            openLoadPopup_ = true;
        }
        ImGui::SameLine(0.0f, btnGap);
        if (menuButton(ICON_FA_GEAR "  Settings", ImVec2(btnW, btnH))) req.action = MenuAction::Settings;
        ImGui::SameLine(0.0f, btnGap);
        if (menuButton(ICON_FA_POWER_OFF "  Quit", ImVec2(btnW, btnH), false, true)) req.action = MenuAction::Quit;
        y += btnH + 16.0f * k;

        if (!message_.empty()) {
            ImGui::SetCursorPos(ImVec2(left, y));
            ImGui::PushTextWrapPos(left + contentW);
            ImGui::TextColored(messageIsError_ ? pal.danger : pal.textDim, "%s", message_.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::EndDisabled();

        // ---- footer hint ----
        {
            const char* hint = "In a scene, Esc opens this menu again.";
            const ImVec2 sz = ImGui::CalcTextSize(hint);
            dl->AddText(ImVec2(0.5f * (display.x - sz.x), display.y - sz.y - 14.0f * k), u32(pal.textDim), hint);
        }

        // ---- the Load scene file... popup ----
        if (openLoadPopup_) {
            ImGui::OpenPopup("Load scene file");
            openLoadPopup_ = false;
        }
        ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, 0.5f * display.y), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSizeConstraints(ImVec2(360.0f * k, 0.0f), ImVec2(520.0f * k, 520.0f * k));
        if (ImGui::BeginPopupModal("Load scene file", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
            ImGui::TextColored(pal.textDim, "Scene files in the %s folder", app::defaultSceneDir().c_str());
            ImGui::Spacing();
            if (files_.empty()) ImGui::TextColored(pal.textDim, "No scene files found.");
            if (ImGui::BeginChild("##files", ImVec2(0.0f, std::min(static_cast<float>(files_.size()), 10.0f) * ImGui::GetFrameHeightWithSpacing()),
                                  ImGuiChildFlags_None)) {
                for (const std::string& f : files_) {
                    if (ImGui::Selectable(f.c_str())) {
                        req.action = MenuAction::LoadFile;
                        req.arg = app::defaultSceneDir() + "/" + f;
                        ImGui::CloseCurrentPopup();
                    }
                }
            }
            ImGui::EndChild();
            ImGui::Spacing();
            if (ImGui::Button(ICON_FA_XMARK "  Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }
    ImGui::End();
    return req;
}

MenuRequest MainMenu::drawPause(ImVec2 display, ImFont* titleFont, bool blocked)
{
    MenuRequest req;
    const Palette& pal = palette();
    const float k = uiScale();

    // The dimmed scene: a full-window layer that also keeps clicks away from the windows below it
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(display);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.02f, 0.50f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##pausedim", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();

    const float width = 340.0f * k;
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, 0.5f * display.y), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(width, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, withAlpha(pal.panel, 0.97f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22.0f * k, 18.0f * k));
    const bool visible = ImGui::Begin("##pausemenu", nullptr,
                                      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                          ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    if (visible) {
        centredText(wasRunning_ ? "Paused" : "Menu", titleFont, 32.0f, pal.text);
        if (!sceneName_.empty()) centredText(sceneName_.c_str(), nullptr, 0.0f, pal.textDim);
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::BeginDisabled(blocked);
        const ImVec2 btn(-FLT_MIN, 44.0f * k);
        if (menuButton(ICON_FA_PLAY "  Resume", btn, true)) req.action = MenuAction::Resume;
        if (menuButton(ICON_FA_GEAR "  Settings", btn)) req.action = MenuAction::Settings;
        if (menuButton(ICON_FA_HOUSE "  Return to menu", btn)) req.action = MenuAction::ReturnToMenu;
        if (menuButton(ICON_FA_POWER_OFF "  Quit", btn, false, true)) req.action = MenuAction::Quit;
        ImGui::EndDisabled();
        ImGui::Spacing();
        centredText("Esc resumes", nullptr, 0.0f, pal.textDim);
    }
    ImGui::End();
    return req;
}

} // namespace ui
} // namespace cosmos
