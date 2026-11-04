#pragma once
// The Settings window (V1.1 step 1), opened from the start screen and from the Esc menu. A centred window with five tabs:
//   Graphics | Simulation defaults | Display & units | Controls | Credits
// It edits an app::Settings in place and says when something changed; the application applies and saves the settings (see
// main.cpp). The Controls and Credits text lives in info_text.cpp.
#include "app/settings.h"
#include "imgui.h"

namespace cosmos {
namespace ui {

enum class SettingsTab { Graphics, Simulation, Display, Controls, Credits };

// What the window needs to know besides the settings themselves.
struct SettingsPanelInfo {
    int launchMsaa = 4;               // the sample count this run's window was created with (a different choice needs a restart)
    bool uiScaleLocked = false;       // --ui-scale was given: the saved UI scale is not used in this run
    bool vsyncLocked = false;         // --no-vsync was given
    int hardwareThreads = 1;          // for the "Automatic" thread count
    const char* settingsFile = "settings.json";
};

class SettingsPanel {
public:
    // Opens the window on a tab (or brings it to the front).
    void show(SettingsTab tab = SettingsTab::Graphics);
    void close() { open_ = false; }
    bool isOpen() const { return open_; }

    // Draws the window while it is open. Call between UiContext::newFrame() and render(). `emphasis` is the medium-weight font.
    // Returns true when a setting was changed this frame (the caller applies and saves).
    bool draw(app::Settings& settings, ImVec2 display, const SettingsPanelInfo& info, ImFont* emphasis = nullptr);

private:
    bool drawGraphics(app::Settings& s, const SettingsPanelInfo& info);
    bool drawSimulation(app::Settings& s, const SettingsPanelInfo& info);
    bool drawDisplay(app::Settings& s);
    void drawControls();
    void drawCredits(ImFont* emphasis);

    bool open_ = false;
    int pendingTab_ = -1;        // tab to select on the next frame
    float scaleEdit_ = 1.0f;     // the UI scale slider's value while it is dragged (applied on release: the slider itself rescales)
    bool scaleActive_ = false;
};

} // namespace ui
} // namespace cosmos
