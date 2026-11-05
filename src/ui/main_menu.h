#pragma once
// The start screen and the Esc (pause) menu (V1.1 step 1).
//
//   Start screen   shown at launch when no --scene is given: the title, one card per scene preset (Lucerna, Real solar system,
//                  Empty sandbox), and the buttons Load scene file..., Settings and Quit. It covers the whole window; nothing of
//                  the scene or the editor is drawn behind it (the app draws the starless background only).
//   Pause menu     a centred window over the dimmed scene while a scene is open: Resume, Settings, Return to menu, Quit.
//
// The menu only draws and reports what was chosen (MenuRequest); main.cpp does the work (load a scene, pause and resume the
// run, quit) and opens the Settings window, which sits above the menu.
#include <string>
#include <vector>
#include "imgui.h"

namespace cosmos {
namespace ui {

enum class MenuMode { Hidden, Start, Pause };

enum class MenuAction {
    None,
    NewPreset,      // arg = the preset id
    LoadFile,       // arg = the path of the scene file
    Settings,
    Resume,
    ReturnToMenu,
    Quit,
};

struct MenuRequest {
    MenuAction action = MenuAction::None;
    std::string arg;
    // From the "Discard unsaved changes?" dialog: the user agreed (Discard or Save), do the action without asking again.
    bool confirmed = false;
    bool saveFirst = false;   // the user chose Save: save the scene first, and do the action only if that worked
};

class MainMenu {
public:
    MenuMode mode() const { return mode_; }
    bool active() const { return mode_ != MenuMode::Hidden; }

    void showStart();
    // `sceneName` is shown under the title; `wasRunning`: the run was Running when the menu opened (the title says Paused).
    void showPause(const std::string& sceneName, bool wasRunning);
    void hide();
    // A line under the buttons of the start screen (for example why a scene could not be loaded).
    void setMessage(const std::string& text, bool isError);

    // ---- "Discard unsaved changes?" (V1.1 step 1) --------------------------------------------------------------------
    // Opens the dialog for an action that would lose unsaved edits (ReturnToMenu or Quit). `savePath` is where Save writes
    // (shown in its tooltip). drawConfirm then returns the action with confirmed = true on Discard, confirmed and
    // saveFirst on Save, and nothing on Cancel (or cancelConfirm, which Esc calls).
    void askDiscard(MenuAction then, const std::string& savePath);
    bool confirming() const { return confirmThen_ != MenuAction::None; }
    void cancelConfirm() { confirmThen_ = MenuAction::None; }
    // Draws the dialog while confirming(). Call every frame after draw() (also when the menu is hidden: Ctrl+Q and closing the
    // window ask from the scene) and outside any window.
    MenuRequest drawConfirm(ImVec2 display, ImFont* titleFont);

    // Draws the menu if it is active. Call between UiContext::newFrame() and render(), after the other windows (so it is on top of
    // them). `titleFont` is the medium-weight font. `blocked`: the Settings window is open, the menu's buttons do nothing.
    MenuRequest draw(ImVec2 display, ImFont* titleFont, bool blocked);

private:
    MenuRequest drawStart(ImVec2 display, ImFont* titleFont, bool blocked);
    MenuRequest drawPause(ImVec2 display, ImFont* titleFont, bool blocked);

    MenuMode mode_ = MenuMode::Hidden;
    std::string sceneName_;
    bool wasRunning_ = false;
    std::string message_;
    bool messageIsError_ = false;
    bool openLoadPopup_ = false;
    std::vector<std::string> files_;   // the scene files listed by the Load scene file... popup
    MenuAction confirmThen_ = MenuAction::None;
    bool confirmOpen_ = false;          // OpenPopup was called for the current question
    std::string confirmSavePath_;
};

} // namespace ui
} // namespace cosmos
