#pragma once
// Scene presets and user scene files (plan sections 11.1, 11.2, 10.3). No OpenGL, no ImGui.
//
// Presets are the entries of the "new scene" menu: Empty sandbox (built in) and Lucerna (scenes/lucerna.json). The real
// solar system is added in M8 (plan 11.2): append one entry to the table in presets.cpp (id "solar_system", file
// "solar_system.json") or call registerPreset at start-up; nothing else in the editor or the UI needs to change, because
// they list presets() and load through loadPreset. A preset whose file is missing is reported by presetAvailable() so a
// menu can grey it out.
//
// User scenes are plain scene JSON (core/scene_json.h) at a path the user chooses.
#include <string>
#include <vector>
#include "core/scene.h"

namespace cosmos {
namespace app {

struct PresetInfo {
    std::string id;            // "empty", "lucerna"
    std::string name;          // menu text
    std::string description;
    std::string file;          // scene file inside the scene directory; empty for a built-in generator
    Scene (*build)() = nullptr;   // the built-in generator when file is empty
};

// The empty sandbox: no bodies, no shuttles, epoch 0, default settings. Every body added to it is named "<Kind> <n>".
Scene emptyScene();

// In menu order. The reference stays valid until registerPreset is called.
const std::vector<PresetInfo>& presets();
const PresetInfo* findPreset(const std::string& id);
// Adds a preset (or replaces the one with the same id). For M8's solar system and for tests.
void registerPreset(const PresetInfo& preset);
// The scene directory next to the executable (the build copies scenes/ there)
std::string defaultSceneDir();
// A built-in preset is always available; a file preset when its file can be opened in sceneDir.
bool presetAvailable(const PresetInfo& preset, const std::string& sceneDir);

// Builds or loads a preset. False with a message for an unknown id, a missing or invalid file.
bool loadPreset(const std::string& id, const std::string& sceneDir, Scene& out, std::string& error);

// ---- user scene files ----------------------------------------------------------------------------------------------
// ".json" is appended when the path has no extension.
std::string withSceneExtension(const std::string& path);
// A path that does not name an existing file: `path` (with the extension added) itself when it is free, otherwise the first
// free "<stem>_2.json", "<stem>_3.json", ... next to it (a trailing "_<n>" of the stem is replaced, so saving "a_2.json"
// again gives "a_3.json" and not "a_2_2.json"). Saving through this never overwrites a file, the shipped presets included.
std::string uniqueScenePath(const std::string& path);
// Writes the scene (creating missing parent folders). False with a message on failure.
bool saveSceneToPath(const std::string& path, const Scene& scene, std::string& error);
// Loads and validates a scene file; an empty scene name becomes the file name without extension.
bool loadSceneFromPath(const std::string& path, Scene& out, std::string& error);
// Names of the *.json files in a folder, sorted (empty if the folder does not exist).
std::vector<std::string> listSceneFiles(const std::string& dir);

} // namespace app
} // namespace cosmos
