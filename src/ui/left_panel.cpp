#include "ui/left_panel.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <fstream>
#include <set>
#include "IconsFontAwesome7.h"
#include "app/presets.h"
#include "app/units.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

using app::EditResult;
using app::EditorModel;
using app::Field;
using app::FieldInfo;
using app::ObjectRow;
using app::ObjectType;
using app::Quantity;

namespace {

constexpr BodyKind kAllKinds[] = {BodyKind::Star,        BodyKind::Planet,   BodyKind::GasGiant,   BodyKind::DwarfPlanet,
                                  BodyKind::Moon,        BodyKind::Asteroid, BodyKind::NeutronStar, BodyKind::BlackHole};
// The kinds of the "add" menus (plan 12.1: Dwarf Planet is only a Lucerna display kind)
constexpr BodyKind kAddKinds[] = {BodyKind::Star,     BodyKind::Planet,       BodyKind::GasGiant, BodyKind::Moon,
                                  BodyKind::Asteroid, BodyKind::NeutronStar, BodyKind::BlackHole};
constexpr ControllerKind kControllers[] = {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde,
                                           ControllerKind::Wander};

float uiScale() { return ImGui::GetFontSize() / 16.0f; }

void tip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
}

void dim(const char* text) { ImGui::TextColored(palette().textDim, "%s", text); }

void wrapped(const ImVec4& colour, const std::string& text)
{
    ImGui::PushStyleColor(ImGuiCol_Text, colour);
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
}

// Edits a std::string in a fixed buffer (no imgui_stdlib). Returns what InputText returns.
bool inputString(const char* id, std::string& s, ImGuiInputTextFlags flags = 0)
{
    char buf[512];
    std::strncpy(buf, s.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    const bool r = ImGui::InputText(id, buf, sizeof(buf), flags);
    if (std::strcmp(buf, s.c_str()) != 0) s = buf;
    return r;
}

std::string formatLive(double v, Quantity q) { return app::formatQuantity(v, q, nullptr, 4); }

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// Actions

bool LeftPanel::selectObject(std::uint64_t id)
{
    scrollToSelected_ = true;
    return rc_.editor().select(id);
}

void LeftPanel::sceneReplaced(const std::string& message)
{
    fields_.clear();
    fieldsFor_ = 0;
    sticky_.clear();
    actionError_.clear();
    sceneMsg_ = message;
    sceneMsgError_ = false;
    if (hooks.say) hooks.say(message);
    if (hooks.sceneReplaced) hooks.sceneReplaced();
}

bool LeftPanel::newScene(const std::string& presetId)
{
    Scene scene;
    std::string err;
    if (!app::loadPreset(presetId, app::defaultSceneDir(), scene, err)) {
        sceneMsg_ = err;
        sceneMsgError_ = true;
        if (hooks.say) hooks.say(err);
        return false;
    }
    const std::string name = scene.name;
    if (!rc_.loadScene(std::move(scene), err)) {
        sceneMsg_ = err;
        sceneMsgError_ = true;
        if (hooks.say) hooks.say(err);
        return false;
    }
    sceneReplaced("New scene: " + (name.empty() ? presetId : name));
    return true;
}

bool LeftPanel::loadSceneFile(const std::string& pathIn)
{
    const std::string path = app::withSceneExtension(pathIn);
    Scene scene;
    std::string err;
    if (!app::loadSceneFromPath(path, scene, err) || !rc_.loadScene(std::move(scene), err)) {
        sceneMsg_ = "Cannot load " + path + ": " + err;
        sceneMsgError_ = true;
        if (hooks.say) hooks.say(sceneMsg_);
        return false;
    }
    sceneReplaced("Loaded " + path + " (" + rc_.editor().scene().name + ")");
    return true;
}

bool LeftPanel::saveCurrent(const std::string& pathIn)
{
    const std::string& base = pathIn.empty() ? scenePath : pathIn;
    if (base.empty()) {
        sceneMsg_ = "Cannot save: enter a file name";
        sceneMsgError_ = true;
        if (hooks.say) hooks.say(sceneMsg_);
        return false;
    }
    const std::string wanted = app::withSceneExtension(base);
    // Never overwrite: an existing file (an earlier save, a shipped preset) keeps its content and the new one gets a free
    // name next to it (my_scene.json -> my_scene_2.json).
    const std::string path = app::uniqueScenePath(wanted);
    std::string err;
    if (!rc_.saveCurrent(path, err)) {
        sceneMsg_ = "Cannot save " + path + ": " + err;
        sceneMsgError_ = true;
        if (hooks.say) hooks.say(sceneMsg_);
        return false;
    }
    sceneMsg_ = "Saved " + path;
    if (path != wanted) sceneMsg_ += " (" + wanted + " already exists, it was not overwritten)";
    sceneMsgError_ = false;
    if (pathIn.empty()) scenePath = path;   // Load then reads what was just saved
    if (hooks.say) hooks.say(sceneMsg_);
    return true;
}

bool LeftPanel::addBody(BodyKind kind)
{
    EditorModel& ed = rc_.editor();
    const bool first = ed.scene().bodies.empty();
    const Vec3 at = hooks.spawnPosition ? hooks.spawnPosition() : Vec3{};
    const EditResult r = ed.addBody(kind, at, Vec3{});
    if (!r) {
        actionError_ = r.error;
        if (hooks.say) hooks.say(r.error);
        return false;
    }
    fields_.clear();
    fieldsFor_ = ed.selected();
    fieldsRevision_ = ed.revision();
    sticky_ = r.warnings;
    actionError_.clear();
    scrollToSelected_ = true;
    stickyRevision_ = ed.revision();
    if (hooks.say) hooks.say("Added " + ed.body(r.id)->name);
    if (first && hooks.focus) hooks.focus(r.id, false, true);   // the first object of an empty scene: look at it
    return true;
}

bool LeftPanel::addShuttle(ControllerKind controller)
{
    EditorModel& ed = rc_.editor();
    const Vec3 at = hooks.spawnPosition ? hooks.spawnPosition() : Vec3{};
    const EditResult r = ed.addShuttle(controller, at, Vec3{});
    if (!r) {
        actionError_ = r.error;
        if (hooks.say) hooks.say(r.error);
        return false;
    }
    fields_.clear();
    fieldsFor_ = ed.selected();
    fieldsRevision_ = ed.revision();
    sticky_ = r.warnings;
    actionError_.clear();
    scrollToSelected_ = true;
    stickyRevision_ = ed.revision();
    if (hooks.say) hooks.say("Added " + ed.shuttle(r.id)->name);
    return true;
}

bool LeftPanel::deleteSelected()
{
    EditorModel& ed = rc_.editor();
    const std::uint64_t id = ed.selected();
    if (id == 0) return false;
    std::string name = ed.body(id) ? ed.body(id)->name : ed.shuttle(id) ? ed.shuttle(id)->name : "object";
    const EditResult r = ed.remove(id);
    if (!r) {
        actionError_ = r.error;
        if (hooks.say) hooks.say(r.error);
        return false;
    }
    fields_.clear();
    fieldsFor_ = ed.selected();
    fieldsRevision_ = ed.revision();
    sticky_ = r.warnings;
    stickyRevision_ = ed.revision();
    actionError_.clear();
    if (hooks.say) hooks.say("Deleted " + name);
    return true;
}

// ---------------------------------------------------------------------------------------------------------------
// Drawing

void LeftPanel::draw(ToolbarState& toolbar, ImVec2 display, float barHeight)
{
    occupied_.clear();
    width_ = 0.0f;
    if (toolbar.edit) drawEditPanel(toolbar, display, barHeight);
    if (toolbar.add) drawAddWindow(toolbar, display);
}

void LeftPanel::drawEditPanel(ToolbarState& toolbar, ImVec2 display, float barHeight)
{
    const Palette& pal = palette();
    const float s = uiScale();
    width_ = std::min(380.0f * s, display.x * 0.5f);
    displayHeightDp_ = display.y / s;
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(width_, std::max(60.0f, display.y - barHeight)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    const bool open = ImGui::Begin("Scene editor", &toolbar.edit, flags);
    ImGui::PopStyleVar();
    if (open) {
        const ImVec2 p = ImGui::GetWindowPos(), z = ImGui::GetWindowSize();
        occupied_.push_back(ImVec4(p.x, p.y, p.x + z.x, p.y + z.y));

        // header: scene name and run state
        const app::RunState rs = rc_.state();
        const char* stateText = rs == app::RunState::Running ? "Running" : rs == app::RunState::Paused ? "Paused" : "Stopped";
        const ImVec4 stateColour = rs == app::RunState::Running ? pal.good : rs == app::RunState::Paused ? pal.warn : pal.textDim;
        const std::string& sceneName = rc_.editor().scene().name;
        ImGui::TextUnformatted(sceneName.empty() ? "Untitled scene" : sceneName.c_str());
        ImGui::SameLine(ImGui::GetContentRegionMax().x - ImGui::CalcTextSize(stateText).x - 2.0f * s);
        ImGui::TextColored(stateColour, "%s", stateText);
        if (!rc_.lastError().empty()) wrapped(pal.danger, rc_.lastError());

        drawSceneSection();
        ImGui::Spacing();
        drawObjectButtons();
        const float avail = ImGui::GetContentRegionAvail().y;
        const float rowH = ImGui::GetFrameHeight();
        const std::size_t rows = rc_.listObjects().size();
        const float pad = 2.0f * ImGui::GetStyle().WindowPadding.y;
        const float wantH = static_cast<float>(std::max<std::size_t>(rows, 1)) * rowH + pad + 2.0f;
        drawObjectList(std::min(std::max(wantH, 4.0f * rowH + pad), std::max(avail * 0.34f, 4.0f * rowH)));
        drawSelected();
    }
    ImGui::End();
}

void LeftPanel::drawSceneSection()
{
    const Palette& pal = palette();
    if (!sceneSectionSet_) {   // a short window starts with the scene section folded (the list and fields need the room)
        ImGui::SetNextItemOpen(displayHeightDp_ >= 700.0f, ImGuiCond_Always);
        sceneSectionSet_ = true;
    }
    if (!ImGui::CollapsingHeader("Scene")) return;

    ImGui::SetNextItemWidth(-FLT_MIN);
    inputString("##scenepath", scenePath);
    tip("Path of a scene file. Load reads it; Save writes the current state to it (\".json\" is added, folders are created).");

    if (ImGui::Button(ICON_FA_FILE "  New " ICON_FA_CARET_DOWN)) ImGui::OpenPopup("##newscene");
    tip("Start a new scene from a preset");
    if (ImGui::BeginPopup("##newscene")) {
        for (const app::PresetInfo& p : app::presets()) {
            const bool available = app::presetAvailable(p, app::defaultSceneDir());
            ImGui::BeginDisabled(!available);
            if (ImGui::MenuItem(p.name.c_str())) newScene(p.id);
            ImGui::EndDisabled();
            tip(available ? p.description.c_str() : "The scene file of this preset is missing.");
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FOLDER_OPEN "  Load")) loadSceneFile(scenePath);
    tip("Load the scene file at the path (replaces the scene and stops the run)");
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_CARET_DOWN "##browse")) {
        files_ = app::listSceneFiles(app::defaultSceneDir());
        ImGui::OpenPopup("##browse");
    }
    tip("Scene files in the scenes folder");
    if (ImGui::BeginPopup("##browse")) {
        if (files_.empty()) dim("No scene files found.");
        for (const std::string& f : files_)
            if (ImGui::MenuItem(f.c_str())) scenePath = app::defaultSceneDir() + "/" + f;
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FLOPPY_DISK "  Save")) saveCurrent();
    tip("Save the current state (with mid-run edits) as a scene file at the path");

    if (!sceneMsg_.empty()) wrapped(sceneMsgError_ ? pal.danger : pal.textDim, sceneMsg_);
}

void LeftPanel::drawObjectButtons()
{
    const bool canEdit = rc_.canEdit();
    ImGui::SeparatorText("Objects");
    ImGui::BeginDisabled(!canEdit);
    if (ImGui::Button(ICON_FA_PLUS "  Body " ICON_FA_CARET_DOWN)) ImGui::OpenPopup("##addbody");
    if (!canEdit) tip(app::kLockedMessage); else tip("Add a body of a kind at the centre of the view");
    if (ImGui::BeginPopup("##addbody")) {
        for (BodyKind k : kAddKinds)
            if (ImGui::MenuItem(kindName(k))) addBody(k);
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_ROCKET "  Shuttle")) addShuttle(ControllerKind::Coast);
    if (!canEdit) tip(app::kLockedMessage); else tip("Add a shuttle (Coast controller) at the centre of the view");
    ImGui::SameLine();
    ImGui::BeginDisabled(rc_.editor().selected() == 0);
    if (ImGui::Button(ICON_FA_TRASH "  Delete")) deleteSelected();
    ImGui::EndDisabled();
    if (canEdit) tip("Delete the selected object");
    ImGui::EndDisabled();
}

void LeftPanel::drawObjectList(float height)
{
    const Palette& pal = palette();
    const float s = uiScale();
    EditorModel& ed = rc_.editor();
    if (!ImGui::BeginChild("##objects", ImVec2(0.0f, height), ImGuiChildFlags_Borders)) {
        ImGui::EndChild();
        return;
    }
    const std::vector<ObjectRow> rows = rc_.listObjects();
    if (rows.empty()) dim("The scene is empty. Add a body with + Body.");
    ImDrawList* dl = ImGui::GetWindowDrawList();
    bool scrolled = false;
    for (const ObjectRow& row : rows) {
        const bool sel = row.id == ed.selected();
        ImGui::PushID(static_cast<int>(row.id));
        if (ImGui::Selectable("##row", sel, ImGuiSelectableFlags_None, ImVec2(0.0f, ImGui::GetTextLineHeight() + 4.0f * s))) {
            selectObject(row.id);
            if (hooks.focus) hooks.focus(row.id, row.type == ObjectType::Shuttle, true);
        }
        if (sel && scrollToSelected_ && !scrolled) {
            ImGui::SetScrollHereY(0.5f);
            scrolled = true;
        }
        const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
        const float cy = 0.5f * (a.y + b.y);
        ImVec4 dot = pal.textDim;
        if (row.type == ObjectType::Body)
            if (const SceneBody* body = ed.body(row.id)) dot = ImVec4(body->colour[0], body->colour[1], body->colour[2], 1.0f);
        if (row.type == ObjectType::Body) dl->AddCircleFilled(ImVec2(a.x + 12.0f * s, cy), 5.0f * s, u32(dot));
        else dl->AddText(ImVec2(a.x + 6.0f * s, cy - 0.5f * ImGui::GetTextLineHeight()), u32(dot), ICON_FA_ROCKET);
        const std::string kind = row.type == ObjectType::Body ? kindName(row.kind) : row.controller;
        const float kw = ImGui::CalcTextSize(kind.c_str()).x;
        dl->PushClipRect(a, ImVec2(b.x - kw - 14.0f * s, b.y), true);
        dl->AddText(ImVec2(a.x + 26.0f * s, cy - 0.5f * ImGui::GetTextLineHeight()), u32(pal.text), row.name.c_str());
        dl->PopClipRect();
        dl->AddText(ImVec2(b.x - kw - 8.0f * s, cy - 0.5f * ImGui::GetTextLineHeight()), u32(pal.textDim), kind.c_str());
        ImGui::PopID();
    }
    if (scrolled || rows.empty()) scrollToSelected_ = false;
    ImGui::EndChild();
}

void LeftPanel::resetFieldsIfStale()
{
    EditorModel& ed = rc_.editor();
    if (ed.selected() != fieldsFor_) {
        fields_.clear();
        sticky_.clear();
        actionError_.clear();
        fieldsFor_ = ed.selected();
        fieldsRevision_ = ed.revision();
    } else if (ed.revision() != fieldsRevision_) {
        fields_.clear();
        fieldsRevision_ = ed.revision();
    }
}

void LeftPanel::report(const EditResult& r, std::uint64_t id)
{
    (void)id;
    if (r.ok) {
        actionError_.clear();
        sticky_ = r.warnings;
        stickyRevision_ = rc_.editor().revision();
    } else {
        actionError_ = r.error;
    }
}

void LeftPanel::drawSelected()
{
    const Palette& pal = palette();
    EditorModel& ed = rc_.editor();
    ImGui::SeparatorText("Selected object");
    if (!ImGui::BeginChild("##props", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None)) {
        ImGui::EndChild();
        return;
    }
    const std::uint64_t id = ed.selected();
    const ObjectType type = ed.typeOf(id);
    if (type == ObjectType::None) {
        dim("Select an object in the list or click one in the view.");
        ImGui::EndChild();
        return;
    }
    resetFieldsIfStale();
    const bool running = rc_.running();
    if (running) {
        wrapped(pal.warn, std::string(ICON_FA_LOCK "  ") + "Running: editing is off. Pause or Stop to edit; the values below are live.");
    } else if (rc_.state() == app::RunState::Paused) {
        wrapped(pal.textDim, "Paused: an edit rebuilds the simulation from this moment.");
    }
    if (!actionError_.empty()) wrapped(pal.danger, actionError_);
    drawWarnings(id);

    const bool enabled = !running;
    field(id, Field::Name, enabled);
    if (type == ObjectType::Body) {
        drawBodyFields(id, enabled);
        drawPlacement(id, enabled);
    } else {
        drawShuttleFields(id, enabled);
    }
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::EndChild();
}

void LeftPanel::drawWarnings(std::uint64_t id)
{
    const Palette& pal = palette();
    std::vector<std::string> shown;
    if (stickyRevision_ == rc_.editor().revision()) shown = sticky_;   // the warnings of the last edit, while nothing changed since
    if (!rc_.running()) {
        const EditorModel& ed = rc_.editor();
        auto add = [&](const std::string& w) {
            if (!w.empty() && std::find(shown.begin(), shown.end(), w) == shown.end()) shown.push_back(w);
        };
        for (const std::string& w : ed.placementWarnings(id)) add(w);
        // Plan 10.1: a body above 10% of the heaviest one's mass is flagged for as long as it is. The heaviest body itself is
        // not (the model compares it with the second heaviest, which would flag every star).
        if (const SceneBody* b = ed.body(id)) {
            bool heaviest = true;
            for (const SceneBody& o : ed.scene().bodies)
                if (o.id != id && o.mu > b->mu) heaviest = false;
            if (!heaviest) add(ed.dominanceWarning(id));
        }
    }
    for (const std::string& w : shown) wrapped(pal.warn, std::string(ICON_FA_TRIANGLE_EXCLAMATION "  ") + w);
}

bool LeftPanel::liveText(std::uint64_t id, Field f, std::string& out) const
{
    const app::DisplayState& st = rc_.driver().state();
    if (const app::DisplayBody* b = [&]() -> const app::DisplayBody* {
            for (const app::DisplayBody& x : st.bodies) if (x.id == id) return &x;
            return nullptr; }()) {
        switch (f) {
        case Field::Name: out = b->name; return true;
        case Field::Mass: out = formatLive(b->mu, Quantity::BodyMass); return true;
        case Field::Radius: out = formatLive(b->radius, Quantity::Length); return true;
        case Field::PosX: out = formatLive(b->r.x, Quantity::Length); return true;
        case Field::PosY: out = formatLive(b->r.y, Quantity::Length); return true;
        case Field::VelX: out = formatLive(b->v.x, Quantity::Speed); return true;
        case Field::VelY: out = formatLive(b->v.y, Quantity::Speed); return true;
        default: return false;
        }
    }
    for (const app::DisplayShuttle& s : st.shuttles) {
        if (s.id != id) continue;
        const Shuttles& sh = rc_.driver().sim().shuttles();
        switch (f) {
        case Field::PosX: out = formatLive(s.r.x, Quantity::Length); return true;
        case Field::PosY: out = formatLive(s.r.y, Quantity::Length); return true;
        case Field::VelX: out = formatLive(s.v.x, Quantity::Speed); return true;
        case Field::VelY: out = formatLive(s.v.y, Quantity::Speed); return true;
        case Field::FuelMass:
            if (s.index < sh.fuelMass.size()) { out = formatLive(sh.fuelMass[s.index], Quantity::Mass); return true; }
            return false;
        default: return false;
        }
    }
    return false;
}

void LeftPanel::field(std::uint64_t id, Field f, bool enabled, const char* labelOverride)
{
    const Palette& pal = palette();
    EditorModel& ed = rc_.editor();
    const FieldInfo info = app::fieldInfo(f);
    FieldEdit& st = fields_[static_cast<int>(f)];
    if (!st.editing) {
        std::string live;
        if (rc_.running() && liveText(id, f, live)) st.text = live;
        else st.text = ed.fieldText(id, f);
    }

    ImGui::PushID(static_cast<int>(f));
    const float labelW = ImGui::GetContentRegionAvail().x * 0.40f;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(labelOverride ? labelOverride : info.label);
    ImGui::SameLine(labelW);
    ImGui::SetNextItemWidth(-FLT_MIN);

    char buf[256];
    std::strncpy(buf, st.text.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    const bool bad = !st.error.empty();
    if (bad) {
        ImGui::PushStyleColor(ImGuiCol_Border, pal.danger);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    }
    ImGui::BeginDisabled(!enabled);
    const bool enter = ImGui::InputText("##v", buf, sizeof(buf), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
    const bool left = ImGui::IsItemDeactivatedAfterEdit();
    const bool escaped = ImGui::IsItemDeactivated() && ImGui::IsKeyPressed(ImGuiKey_Escape, false);
    const bool active = ImGui::IsItemActive();
    ImGui::EndDisabled();
    if (bad) {
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }
    if (info.quantity != Quantity::Number && !active && ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled)) {
        std::string t = "Units: " + app::unitList(info.quantity);
        if (info.defaultUnit && *info.defaultUnit) t += "\nA bare number is in " + std::string(info.defaultUnit) + ".";
        t += "\nEnter applies the value.";
        ImGui::SetTooltip("%s", t.c_str());
    }
    if (std::strcmp(buf, st.text.c_str()) != 0) {
        st.text = buf;
        st.editing = true;
        st.error.clear();
    }
    if (escaped) {   // Esc gives up the edit: the field shows the model's value again, a stale error goes away
        st.editing = false;
        st.error.clear();
        st.text = ed.fieldText(id, f);
    } else if (enabled && (enter || left)) {
        const EditResult r = applyText(id, f, st.text);
        if (r.ok) {
            st.editing = false;
            st.error.clear();
        } else {
            st.error = r.error;
        }
    }
    if (!st.error.empty()) wrapped(pal.danger, st.error);
    ImGui::PopID();
}

EditResult LeftPanel::applyText(std::uint64_t id, Field f, const std::string& text)
{
    EditorModel& ed = rc_.editor();
    const std::uint64_t before = ed.revision();
    const EditResult r = ed.setFieldText(id, f, text);
    if (r.ok && ed.revision() != before) report(r, id);
    return r;
}

void LeftPanel::colourRow(std::uint64_t id, bool enabled)
{
    EditorModel& ed = rc_.editor();
    const SceneBody* b = ed.body(id);
    if (!b) return;
    const ImVec4 current(b->colour[0], b->colour[1], b->colour[2], 1.0f);
    const float labelW = ImGui::GetContentRegionAvail().x * 0.40f;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Colour");
    ImGui::SameLine(labelW);
    ImGui::BeginDisabled(!enabled);
    if (ImGui::ColorButton("##colour", current, ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoTooltip,
                           ImVec2(ImGui::GetFrameHeight() * 2.0f, ImGui::GetFrameHeight()))) {
        colourEdit_ = b->colour;
        colourFor_ = id;
        colourDirty_ = false;
        ImGui::OpenPopup("##colourpicker");
    }
    ImGui::EndDisabled();
    auto commit = [&]() {
        if (colourDirty_ && colourFor_ == id) {
            const EditResult r = ed.setColour(id, colourEdit_);
            report(r, id);
        }
        colourDirty_ = false;
    };
    if (ImGui::BeginPopup("##colourpicker")) {
        if (ImGui::ColorPicker3("##picker", colourEdit_.data(), ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview))
            colourDirty_ = true;
        if (colourDirty_ && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) commit();   // applied when the mouse is released
        ImGui::EndPopup();
    } else if (colourDirty_) {
        commit();
    }
}

void LeftPanel::drawBodyFields(std::uint64_t id, bool enabled)
{
    EditorModel& ed = rc_.editor();
    const SceneBody* b = ed.body(id);
    if (!b) return;
    const BodyKind kind = b->kind;
    const bool hasAtmosphere = b->atmosphere.has_value();
    const float labelW = ImGui::GetContentRegionAvail().x * 0.40f;

    // kind dropdown
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Kind");
    ImGui::SameLine(labelW);
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::BeginDisabled(!enabled);
    if (ImGui::BeginCombo("##kind", kindName(kind))) {
        for (BodyKind k : kAllKinds) {
            if (ImGui::Selectable(kindName(k), k == kind) && k != kind) report(ed.setKind(id, k, applyKindDefaults_), id);
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::Dummy(ImVec2(0.0f, 0.0f));
    ImGui::SameLine(labelW);
    ImGui::BeginDisabled(!enabled);
    ImGui::Checkbox("Take the kind's defaults", &applyKindDefaults_);
    ImGui::EndDisabled();
    tip("When the kind changes, also set mass, radius and colour to the defaults of the new kind (plan table 12.1)");

    field(id, Field::Mass, enabled);
    field(id, Field::Radius, enabled);
    colourRow(id, enabled);
    field(id, Field::SpinPeriod, enabled);

    bool atmo = hasAtmosphere;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Atmosphere");
    ImGui::SameLine(labelW);
    ImGui::BeginDisabled(!enabled);
    if (ImGui::Checkbox("##atmo", &atmo)) report(ed.enableAtmosphere(id, atmo), id);
    ImGui::EndDisabled();
    if (hasAtmosphere && atmo) {
        field(id, Field::AtmoRho0, enabled);
        field(id, Field::AtmoScaleHeight, enabled);
    }
}

void LeftPanel::drawPlacement(std::uint64_t id, bool enabled)
{
    EditorModel& ed = rc_.editor();
    const ObjectType type = ed.typeOf(id);
    ImGui::SeparatorText("Placement");
    if (rc_.running()) {   // live position and velocity of the run
        field(id, Field::PosX, false);
        field(id, Field::PosY, false);
        field(id, Field::VelX, false);
        field(id, Field::VelY, false);
        return;
    }
    const bool orbit = ed.inOrbitMode(id);
    ImGui::BeginDisabled(!enabled);
    const bool toState = ImGui::RadioButton("Position and velocity", !orbit);
    ImGui::SameLine();
    const bool toOrbit = ImGui::RadioButton("Orbit around", orbit);
    ImGui::EndDisabled();
    if (toState && orbit) report(ed.bakeState(id), id);
    if (toOrbit && !orbit) {
        // the heaviest body that is not the object (and does not orbit it): the first one the model accepts
        std::vector<const SceneBody*> cands;
        for (const SceneBody& o : ed.scene().bodies)
            if (o.id != id) cands.push_back(&o);
        std::sort(cands.begin(), cands.end(), [](const SceneBody* a, const SceneBody* b) { return a->mu > b->mu; });
        if (cands.empty()) {
            actionError_ = "There is no other body to orbit. Add one first.";
        } else {
            EditResult last;
            for (const SceneBody* c : cands) {
                last = ed.setOrbitAround(id, c->id, ed.suggestedOrbitDistance(c->id, type));
                if (last.ok) break;
            }
            report(last, id);
        }
    }
    if (orbit) drawOrbit(id);
    else {
        field(id, Field::PosX, enabled);
        field(id, Field::PosY, enabled);
        field(id, Field::VelX, enabled);
        field(id, Field::VelY, enabled);
    }
}

void LeftPanel::drawOrbit(std::uint64_t id)
{
    const Palette& pal = palette();
    EditorModel& ed = rc_.editor();
    const SceneBody* body = ed.body(id);
    const SceneShuttle* shuttle = ed.shuttle(id);
    const std::optional<SceneOrbit>& orbit = body ? body->orbit : shuttle->orbit;
    if (!orbit) return;
    const std::uint64_t parent = orbit->parent;
    const double a = orbit->a;
    const bool retro = orbit->retrograde;
    const SceneOrbit current = *orbit;
    const float labelW = ImGui::GetContentRegionAvail().x * 0.40f;

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Parent");
    ImGui::SameLine(labelW);
    ImGui::SetNextItemWidth(-FLT_MIN);
    const SceneBody* parentBody = ed.body(parent);
    if (ImGui::BeginCombo("##parent", parentBody ? parentBody->name.c_str() : "?")) {
        for (const SceneBody& o : ed.scene().bodies) {
            if (o.id == id) continue;
            if (ImGui::Selectable((o.name + "##p" + std::to_string(o.id)).c_str(), o.id == parent) && o.id != parent)
                report(ed.setOrbitAround(id, o.id, a), id);
        }
        ImGui::EndCombo();
    }
    field(id, Field::OrbitA, true, "Distance (a)");
    field(id, Field::OrbitE, true);
    field(id, Field::OrbitOmega, true, "Arg. periapsis");
    field(id, Field::OrbitMeanAnomaly, true, "Mean anomaly M");

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Direction");
    ImGui::SameLine(labelW);
    if (ImGui::RadioButton("Prograde", !retro) && retro) {
        SceneOrbit o = current;
        o.retrograde = false;
        report(ed.setOrbit(id, o), id);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Retrograde", retro) && !retro) {
        SceneOrbit o = current;
        o.retrograde = true;
        report(ed.setOrbit(id, o), id);
    }

    // where it starts (read-only; the orbit fields above are what you edit)
    const std::string pos = ed.fieldText(id, Field::PosX) + ",  " + ed.fieldText(id, Field::PosY);
    const std::string vel = ed.fieldText(id, Field::VelX) + ",  " + ed.fieldText(id, Field::VelY);
    ImGui::PushStyleColor(ImGuiCol_Text, pal.textDim);
    ImGui::TextWrapped("Starts at (%s)\nwith velocity (%s)", pos.c_str(), vel.c_str());
    ImGui::PopStyleColor();
}

void LeftPanel::drawShuttleFields(std::uint64_t id, bool enabled)
{
    EditorModel& ed = rc_.editor();
    const SceneShuttle* sh = ed.shuttle(id);
    if (!sh) return;
    ControllerKind current = ControllerKind::Coast;
    parseControllerKind(sh->controller, current);
    const float labelW = ImGui::GetContentRegionAvail().x * 0.40f;

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Controller");
    ImGui::SameLine(labelW);
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::BeginDisabled(!enabled);
    if (ImGui::BeginCombo("##controller", controllerName(current))) {
        for (ControllerKind k : kControllers)
            if (ImGui::Selectable(controllerName(k), k == current) && k != current) report(ed.setController(id, k), id);
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    tip("Coast: no thrust. Prograde / Retrograde: burn along / against the velocity. Wander: random burns.");

    drawPlacement(id, enabled);

    ImGui::SeparatorText("Shuttle");
    const Field fields[] = {Field::DryMass,    Field::FuelMass,        Field::MaxThrust, Field::ExhaustVelocity,
                            Field::CdA,        Field::CollisionRadius, Field::MaxTorque, Field::Inertia};
    for (Field f : fields) field(id, f, enabled);
}

void LeftPanel::drawAddWindow(ToolbarState& toolbar, ImVec2 display)
{
    const Palette& pal = palette();
    const float s = uiScale();
    const float x = (toolbar.edit && width_ > 0.0f ? width_ : 0.0f) + 8.0f * s;
    ImGui::SetNextWindowPos(ImVec2(x, 8.0f * s));
    ImGui::SetNextWindowSizeConstraints(ImVec2(200.0f * s, 0.0f), ImVec2(std::max(200.0f * s, display.x * 0.4f), 1.0e9f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("Add", &toolbar.add, flags)) {
        const ImVec2 p = ImGui::GetWindowPos(), z = ImGui::GetWindowSize();
        occupied_.push_back(ImVec4(p.x, p.y, p.x + z.x, p.y + z.y));
        const bool canEdit = rc_.canEdit();
        if (!canEdit) wrapped(pal.warn, std::string(ICON_FA_LOCK "  ") + app::kLockedMessage);
        else dim("Added at the centre of the view.");
        ImGui::BeginDisabled(!canEdit);
        for (BodyKind k : kAddKinds) {
            const BodyDesc d = defaultBodyDesc(k);
            const ImVec4 c(d.colour[0], d.colour[1], d.colour[2], 1.0f);
            ImGui::ColorButton((std::string("##c") + kindName(k)).c_str(), c, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoAlpha,
                               ImVec2(ImGui::GetFrameHeight() * 0.7f, ImGui::GetFrameHeight() * 0.7f));
            ImGui::SameLine();
            if (ImGui::Selectable(kindName(k), false, ImGuiSelectableFlags_None, ImVec2(160.0f * s, 0.0f))) addBody(k);
        }
        ImGui::Separator();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(ICON_FA_ROCKET);
        ImGui::SameLine();
        if (ImGui::Selectable("Shuttle", false, ImGuiSelectableFlags_None, ImVec2(80.0f * s, 0.0f))) addShuttle(addController_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f * s);
        if (ImGui::BeginCombo("##addcontroller", controllerName(addController_))) {
            for (ControllerKind k : kControllers)
                if (ImGui::Selectable(controllerName(k), k == addController_)) addController_ = k;
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();
    }
    ImGui::End();
}

} // namespace ui
} // namespace cosmos
