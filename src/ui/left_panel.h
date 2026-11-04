#pragma once
// The left panel (plan section 10.1): the scene editor in the Universe Sandbox style.
//
//   +----------------------------------+
//   | Scene     path [.....]  [New v] [Load] [Save]     scene menu: presets, load / save by path
//   | Objects   [+ Body v] [+ Shuttle] [Delete]         every body and shuttle as a text row
//   |   o Lucerna            Star                       click a row: select it and focus the camera
//   |   o Domus              Planet
//   | Selected object                                   typed fields with units, kind, colour, placement
//   |   Name ...  Kind ...  Mass ...  Radius ...
//   |   ( ) Position and velocity  (o) Orbit around [parent] a e omega M prograde/retrograde
//   +----------------------------------+
//
// The panel draws and calls app::RunControl / app::EditorModel; it holds no simulation state of its own. Every field edit
// goes through EditorModel::setFieldText (Enter or leaving the field applies it; the error comes back next to the field
// and the warnings in the panel). Editing is only possible while the run is Paused or Stopped: while Running the fields are
// disabled and show the live values of the simulation, with a hint.
//
// The "add" toolbar button opens a small palette of body kinds and a shuttle next to the panel. The panel needs a few
// things only the application knows (where the camera looks, how to focus it, what to do after a scene was replaced);
// those are the hooks.
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include "app/editor_model.h"
#include "app/run_control.h"
#include "core/vec3.h"
#include "imgui.h"
#include "ui/bottom_bar.h"

namespace cosmos {
namespace ui {

struct LeftPanelHooks {
    // Select-and-look: follow this object with the camera (frame = also zoom to fit it). Called for a list click and for the
    // first object added to an empty scene.
    std::function<void(std::uint64_t id, bool shuttle, bool frame)> focus;
    // One line for the event log.
    std::function<void(const std::string&)> say;
    // After RunControl::loadScene succeeded (preset or file): reset the view, the stats, the lookups.
    std::function<void()> sceneReplaced;
    // Where a new object appears: the world position at the centre of the view.
    std::function<Vec3()> spawnPosition;
};

class LeftPanel {
public:
    explicit LeftPanel(app::RunControl& rc) : rc_(rc) {}

    LeftPanelHooks hooks;
    // Path of the "Load" / "Save" buttons and of the bottom bar's "Save as scene" (".json" is added when missing).
    std::string scenePath = "scenes/my_scene.json";

    // Draws the panel while toolbar.edit is set and the add palette while toolbar.add is set. Call between
    // UiContext::newFrame() and render(). `barHeight` is the bottom bar's height (the panel stops above it).
    void draw(ToolbarState& toolbar, ImVec2 display, float barHeight);

    // Screen rectangles (x0, y0, x1, y1) of the windows drawn last frame (keep-out for the labels).
    const std::vector<ImVec4>& occupied() const { return occupied_; }

    // ---- actions (the widgets call these; so do the scripted dev commands) -----------------------------------------
    // Select an object (the viewport pick) and scroll the list to it. The camera is the caller's business.
    bool selectObject(std::uint64_t id);
    // Replace the document by a preset ("empty", "lucerna", ...) or by a scene file. Stops any run.
    bool newScene(const std::string& presetId);
    bool loadSceneFile(const std::string& path);
    // "Save current as scene" to scenePath (or `path`): the state of the run as it is, or the edited scene when stopped.
    bool saveCurrent(const std::string& path = std::string());
    // Adds at the spawn position, selects and focuses the new object. Logs the failure and returns false.
    bool addBody(BodyKind kind);
    bool addShuttle(ControllerKind controller = ControllerKind::Coast);
    bool deleteSelected();
    // What Enter in a field does: parse the text with its units and apply it (the warnings are kept for the panel). The error
    // of a refused edit is in the result.
    app::EditResult applyText(std::uint64_t id, app::Field f, const std::string& text);

    // The last message of the scene section (for tests and the scripted commands)
    const std::string& sceneMessage() const { return sceneMsg_; }

private:
    struct FieldEdit {
        std::string text;     // what the box holds while the user edits it
        bool editing = false; // the text is the user's, not the model's
        std::string error;
    };

    void drawEditPanel(ToolbarState& toolbar, ImVec2 display, float barHeight);
    void drawAddWindow(ToolbarState& toolbar, ImVec2 display);
    void drawSceneSection();
    void drawObjectButtons();
    void drawObjectList(float height);
    void drawSelected();
    void drawBodyFields(std::uint64_t id, bool enabled);
    void drawShuttleFields(std::uint64_t id, bool enabled);
    void drawPlacement(std::uint64_t id, bool enabled);
    void drawOrbit(std::uint64_t id);
    void drawWarnings(std::uint64_t id);

    // One labelled text field. `live` (optional) replaces the model's text while Running.
    void field(std::uint64_t id, app::Field f, bool enabled, const char* labelOverride = nullptr);
    bool liveText(std::uint64_t id, app::Field f, std::string& out) const;
    void colourRow(std::uint64_t id, bool enabled);
    void report(const app::EditResult& r, std::uint64_t id);   // keep the warnings or show the error
    void resetFieldsIfStale();
    void sceneReplaced(const std::string& message);

    app::RunControl& rc_;
    std::vector<ImVec4> occupied_;

    std::map<int, FieldEdit> fields_;
    std::uint64_t fieldsFor_ = 0;
    std::uint64_t fieldsRevision_ = 0;
    std::vector<std::string> sticky_;   // warnings of the last edit of the selected object
    std::uint64_t stickyRevision_ = 0;  // the editor revision they belong to
    std::string actionError_;           // error of the last non-field edit (kind, placement, add, delete)
    bool applyKindDefaults_ = true;
    bool scrollToSelected_ = false;
    std::array<float, 3> colourEdit_{1.0f, 1.0f, 1.0f};
    bool colourDirty_ = false;
    std::uint64_t colourFor_ = 0;
    ControllerKind addController_ = ControllerKind::Coast;
    std::string sceneMsg_;
    bool sceneMsgError_ = false;
    std::vector<std::string> files_;    // scene files listed when the browse menu opens
    float displayHeightDp_ = 0.0f;      // window height in units of the UI scale
    bool sceneSectionSet_ = false;      // the scene section's initial open state was chosen
    float width_ = 0.0f;                // the left panel's width as drawn (the add palette sits next to it)
};

} // namespace ui
} // namespace cosmos
