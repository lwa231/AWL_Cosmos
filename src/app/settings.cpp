#include "app/settings.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>

namespace cosmos {
namespace app {

namespace {

using json = nlohmann::ordered_json;   // keys keep the order they are written in

template <typename T>
T clampTo(T v, T lo, T hi) { return std::min(hi, std::max(lo, v)); }

// The nearest of the choices; ties go to the larger one.
template <std::size_t N>
int snapToChoice(int v, const int (&choices)[N])
{
    int best = choices[0];
    for (int c : choices)
        if (std::abs(c - v) <= std::abs(best - v)) best = c;
    return best;
}

// ---- names in the file ----
struct LengthName { LengthUnit v; const char* name; };
constexpr LengthName kLengthNames[] = {{LengthUnit::Auto, "auto"}, {LengthUnit::Metres, "m"}, {LengthUnit::Kilometres, "km"}, {LengthUnit::AU, "AU"}};
struct MassName { MassUnit v; const char* name; };
constexpr MassName kMassNames[] = {{MassUnit::Auto, "auto"}, {MassUnit::Kilograms, "kg"}, {MassUnit::EarthMasses, "earth"}, {MassUnit::SolarMasses, "solar"}};
struct DateName { DateFormat v; const char* name; };
constexpr DateName kDateNames[] = {{DateFormat::DayClock, "day_clock"}, {DateFormat::DayOnly, "day"}, {DateFormat::YearDay, "year_day"}};
struct QualityName { Quality v; const char* name; };
constexpr QualityName kQualityNames[] = {{Quality::Low, "low"}, {Quality::Medium, "medium"}, {Quality::High, "high"}};

template <typename Table, typename E>
const char* nameOf(const Table& t, E v)
{
    for (const auto& e : t)
        if (e.v == v) return e.name;
    return t[0].name;
}

template <typename Table, typename E>
void readEnum(const json& obj, const char* key, const Table& t, E& out)
{
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_string()) return;
    const std::string s = it->template get<std::string>();
    for (const auto& e : t)
        if (s == e.name) { out = e.v; return; }
}

void readBool(const json& obj, const char* key, bool& out)
{
    auto it = obj.find(key);
    if (it != obj.end() && it->is_boolean()) out = it->get<bool>();
}

void readInt(const json& obj, const char* key, int& out)
{
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_number()) return;
    const double d = it->get<double>();
    if (std::isfinite(d)) out = static_cast<int>(clampTo(d, -1.0e9, 1.0e9));
}

template <typename T>
void readReal(const json& obj, const char* key, T& out)
{
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_number()) return;
    const double d = it->get<double>();
    if (std::isfinite(d)) out = static_cast<T>(d);
}

const json* section(const json& root, const char* key)
{
    auto it = root.find(key);
    return it != root.end() && it->is_object() ? &*it : nullptr;
}

struct OverlayKey { const char* key; bool StartupOverlays::*member; };
constexpr OverlayKey kOverlayKeys[] = {
    {"names", &StartupOverlays::names}, {"velocityArrows", &StartupOverlays::velocityArrows},
    {"accelerationArrows", &StartupOverlays::accelerationArrows}, {"trails", &StartupOverlays::trails},
    {"osculatingOrbits", &StartupOverlays::osculatingOrbits}, {"predictedPath", &StartupOverlays::predictedPath},
    {"gravityRings", &StartupOverlays::gravityRings}, {"hill", &StartupOverlays::hill}, {"soi", &StartupOverlays::soi},
    {"roche", &StartupOverlays::roche}, {"atmospheres", &StartupOverlays::atmospheres}};

} // namespace

const char* qualityName(Quality q)
{
    switch (q) {
    case Quality::Low: return "Low";
    case Quality::Medium: return "Medium";
    case Quality::High: return "High";
    }
    return "Medium";
}

void clampSettings(Settings& s)
{
    const Settings def;
    s.msaa = snapToChoice(s.msaa, kMsaaChoices);
    s.fpsCap = snapToChoice(s.fpsCap, kFpsCapChoices);
    if (!std::isfinite(s.uiScale)) s.uiScale = def.uiScale;
    if (s.uiScale <= 0.0f) s.uiScale = 0.0f;   // 0 = automatic
    else s.uiScale = clampTo(s.uiScale, kMinUiScale, kMaxUiScale);
    s.threads = clampTo(s.threads, 0, kMaxThreadSetting);
    if (!std::isfinite(s.defaultWarp)) s.defaultWarp = def.defaultWarp;
    s.defaultWarp = clampTo(s.defaultWarp, kMinDefaultWarp, kMaxDefaultWarp);
    if (!std::isfinite(s.controllerTick)) s.controllerTick = def.controllerTick;
    s.controllerTick = clampTo(s.controllerTick, kMinControllerTick, kMaxControllerTick);
}

std::string settingsToJson(const Settings& s)
{
    json g = {{"vsync", s.vsync}, {"msaa", s.msaa}, {"uiScale", s.uiScale}, {"fpsCap", s.fpsCap},
              {"starQuality", nameOf(kQualityNames, s.starQuality)}, {"skyboxQuality", nameOf(kQualityNames, s.skyboxQuality)}};
    json sim = {{"threads", s.threads}, {"defaultWarp", s.defaultWarp}, {"autoSlowDown", s.autoSlowDown},
                {"slowDownAnyShuttle", s.slowDownAnyShuttle}, {"controllerTick", s.controllerTick}};
    json ov = json::object();
    for (const OverlayKey& k : kOverlayKeys) ov[k.key] = s.overlays.*k.member;
    json d = {{"lengthUnit", nameOf(kLengthNames, s.lengthUnit)}, {"massUnit", nameOf(kMassNames, s.massUnit)},
              {"dateFormat", nameOf(kDateNames, s.dateFormat)}, {"overlays", ov}};
    json root = {{"version", 1}, {"graphics", g}, {"simulation", sim}, {"display", d}};
    return root.dump(2) + "\n";
}

bool mergeSettingsJson(Settings& s, const std::string& text)
{
    const json root = json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded() || !root.is_object()) return false;

    if (const json* g = section(root, "graphics")) {
        readBool(*g, "vsync", s.vsync);
        readInt(*g, "msaa", s.msaa);
        readReal(*g, "uiScale", s.uiScale);
        readInt(*g, "fpsCap", s.fpsCap);
        readEnum(*g, "starQuality", kQualityNames, s.starQuality);
        readEnum(*g, "skyboxQuality", kQualityNames, s.skyboxQuality);
    }
    if (const json* sim = section(root, "simulation")) {
        readInt(*sim, "threads", s.threads);
        readReal(*sim, "defaultWarp", s.defaultWarp);
        readBool(*sim, "autoSlowDown", s.autoSlowDown);
        readBool(*sim, "slowDownAnyShuttle", s.slowDownAnyShuttle);
        readReal(*sim, "controllerTick", s.controllerTick);
    }
    if (const json* d = section(root, "display")) {
        readEnum(*d, "lengthUnit", kLengthNames, s.lengthUnit);
        readEnum(*d, "massUnit", kMassNames, s.massUnit);
        readEnum(*d, "dateFormat", kDateNames, s.dateFormat);
        if (const json* ov = section(*d, "overlays"))
            for (const OverlayKey& k : kOverlayKeys) readBool(*ov, k.key, s.overlays.*k.member);
    }
    clampSettings(s);
    return true;
}

Settings settingsFromJson(const std::string& text, bool* ok)
{
    Settings s;
    const bool good = mergeSettingsJson(s, text);
    if (ok) *ok = good;
    return s;
}

std::string defaultSettingsPath() { return "settings.json"; }

bool loadSettings(const std::string& path, Settings& out, std::string& note)
{
    out = Settings();
    note.clear();
    std::ifstream f(path, std::ios::binary);
    if (!f) return true;   // no file yet: first launch
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    bool ok = false;
    out = settingsFromJson(text, &ok);
    if (!ok) {
        note = "settings file " + path + " is not valid JSON; using the defaults";
        return false;
    }
    return true;
}

bool saveSettings(const std::string& path, const Settings& s, std::string& error)
{
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        error = "cannot write " + path;
        return false;
    }
    f << settingsToJson(s);
    f.flush();
    if (!f) {
        error = "error while writing " + path;
        return false;
    }
    return true;
}

void applyDisplayPrefs(const Settings& s)
{
    DisplayPrefs& p = displayPrefs();
    p.length = s.lengthUnit;
    p.mass = s.massUnit;
    p.date = s.dateFormat;
}

} // namespace app
} // namespace cosmos
