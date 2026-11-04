#include "app/presets.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include "core/scene_json.h"

namespace cosmos {
namespace app {

namespace fs = std::filesystem;

Scene emptyScene()
{
    Scene s;
    s.name = "Empty sandbox";
    return s;
}

namespace {

std::vector<PresetInfo>& table()
{
    static std::vector<PresetInfo> t = {
        {"empty", "Empty sandbox", "Nothing in it. Every body you add is named <Kind> <n>.", "", &emptyScene},
        {"lucerna", "Lucerna", "The default template: a KSP-scale system with fictional densities.", "lucerna.json", nullptr},
        // M8: {"solar_system", "Real solar system", "Sun, planets, Moon and Pluto from JPL Horizons.", "solar_system.json", nullptr},
    };
    return t;
}

} // namespace

const std::vector<PresetInfo>& presets() { return table(); }

const PresetInfo* findPreset(const std::string& id)
{
    for (const PresetInfo& p : table())
        if (p.id == id) return &p;
    return nullptr;
}

void registerPreset(const PresetInfo& preset)
{
    for (PresetInfo& p : table()) {
        if (p.id == preset.id) {
            p = preset;
            return;
        }
    }
    table().push_back(preset);
}

std::string defaultSceneDir() { return "scenes"; }

bool presetAvailable(const PresetInfo& preset, const std::string& sceneDir)
{
    if (preset.file.empty()) return preset.build != nullptr;
    std::ifstream f((fs::path(sceneDir) / preset.file).string(), std::ios::binary);
    return static_cast<bool>(f);
}

bool loadPreset(const std::string& id, const std::string& sceneDir, Scene& out, std::string& error)
{
    const PresetInfo* p = findPreset(id);
    if (!p) {
        error = "Unknown preset \"" + id + "\"";
        return false;
    }
    if (p->file.empty()) {
        if (!p->build) {
            error = "Preset \"" + p->name + "\" has no source";
            return false;
        }
        out = p->build();
        return true;
    }
    const std::string path = (fs::path(sceneDir) / p->file).string();
    if (!loadSceneFile(path, out, error)) return false;
    if (out.name.empty()) out.name = p->name;
    return true;
}

std::string withSceneExtension(const std::string& path)
{
    const fs::path p(path);
    if (p.has_extension()) return path;
    return path + ".json";
}

std::string uniqueScenePath(const std::string& path)
{
    const std::string full = withSceneExtension(path);
    std::error_code ec;
    if (!fs::exists(fs::path(full), ec)) return full;
    const fs::path p(full);
    std::string stem = p.stem().string();
    const std::size_t us = stem.rfind('_');
    if (us != std::string::npos && us + 1 < stem.size() &&
        stem.find_first_not_of("0123456789", us + 1) == std::string::npos)
        stem.erase(us);
    for (int n = 2; n < 100000; ++n) {
        const fs::path cand = p.parent_path() / (stem + "_" + std::to_string(n) + p.extension().string());
        if (!fs::exists(cand, ec)) return cand.generic_string();
    }
    return full;
}

bool saveSceneToPath(const std::string& path, const Scene& scene, std::string& error)
{
    if (path.empty()) {
        error = "Enter a file name";
        return false;
    }
    const std::string full = withSceneExtension(path);
    try {
        const fs::path parent = fs::path(full).parent_path();
        if (!parent.empty()) {
            std::error_code ec;
            fs::create_directories(parent, ec);
            if (ec) {
                error = "cannot create the folder \"" + parent.string() + "\": " + ec.message();
                return false;
            }
        }
    } catch (const std::exception& e) {
        error = std::string("cannot create the folder: ") + e.what();
        return false;
    }
    return saveSceneFile(full, scene, error);
}

bool loadSceneFromPath(const std::string& path, Scene& out, std::string& error)
{
    if (!loadSceneFile(path, out, error)) return false;
    if (out.name.empty()) out.name = fs::path(path).stem().string();
    return true;
}

std::vector<std::string> listSceneFiles(const std::string& dir)
{
    std::vector<std::string> names;
    std::error_code ec;
    fs::directory_iterator it(dir, ec), end;
    if (ec) return names;
    for (; it != end; it.increment(ec)) {
        if (ec) break;
        std::error_code e2;
        if (!it->is_regular_file(e2)) continue;
        if (it->path().extension() == ".json") names.push_back(it->path().filename().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

} // namespace app
} // namespace cosmos
