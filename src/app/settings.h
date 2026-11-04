#pragma once
// Application settings (V1.1 step 1): what the Settings window edits and settings.json stores. No OpenGL, no ImGui.
//
// A Settings value is always valid: clampSettings() snaps every field into its range, and loading never fails. A missing file,
// a missing or unknown key, a value of the wrong type, or a file that is not JSON at all leaves the defaults (or the keys that
// were readable) in place, so a damaged settings.json can never stop the program from starting.
//
// File layout (settings.json, in the working folder next to the scenes/ and shaders/ folders):
//   { "version": 1,
//     "graphics":   { "vsync", "msaa", "uiScale", "fpsCap", "starQuality", "skyboxQuality" },
//     "simulation": { "threads", "defaultWarp", "autoSlowDown", "slowDownAnyShuttle", "controllerTick" },
//     "display":    { "lengthUnit", "massUnit", "dateFormat", "overlays": { "names", "velocityArrows", ... } } }
// Unknown keys are ignored (and dropped at the next save).
#include <string>
#include "app/display_prefs.h"

namespace cosmos {
namespace app {

enum class Quality { Low, Medium, High };
const char* qualityName(Quality q);   // "Low", "Medium", "High"

// Overlays that are switched on when the program starts (the View window's checkboxes).
struct StartupOverlays {
    bool names = true;
    bool velocityArrows = false;
    bool accelerationArrows = false;
    bool trails = false;
    bool osculatingOrbits = false;
    bool predictedPath = false;
    bool gravityRings = false;
    bool hill = false;
    bool soi = false;
    bool roche = false;
    bool atmospheres = false;
};

struct Settings {
    // ---- Graphics ----
    bool vsync = true;
    int msaa = 4;                  // samples of the window's framebuffer: 0, 2, 4 or 8. Needs a new context: applies at the next launch
    float uiScale = 0.0f;          // 0 = the window's own content scale; otherwise 0.5 .. 3
    int fpsCap = 0;                // frames per second: 0 (off), 60, 120, 144 or 240
    Quality starQuality = Quality::Medium;     // stored now; the procedural stars of step 3 read it
    Quality skyboxQuality = Quality::Medium;   // likewise

    // ---- Simulation defaults ----
    int threads = 0;               // shuttle-pass worker threads: 0 = hardware concurrency (at most 16), otherwise 1 .. 16
    double defaultWarp = 1.0;      // time rate of a freshly opened scene, 0.01 .. 1e6
    bool autoSlowDown = true;      // step the warp down for an approaching impact
    bool slowDownAnyShuttle = false;   // ... for every shuttle, not only the followed or selected one
    double controllerTick = 0.1;   // seconds between autopilot decisions, for scenes that do not set their own

    // ---- Display & units ----
    LengthUnit lengthUnit = LengthUnit::Auto;
    MassUnit massUnit = MassUnit::Auto;
    DateFormat dateFormat = DateFormat::DayClock;
    StartupOverlays overlays;
};

inline constexpr int kMaxThreadSetting = 16;
inline constexpr double kMinDefaultWarp = 0.01;
inline constexpr double kMaxDefaultWarp = 1.0e6;
inline constexpr double kMinControllerTick = 0.001;
inline constexpr double kMaxControllerTick = 60.0;
inline constexpr float kMinUiScale = 0.5f;
inline constexpr float kMaxUiScale = 3.0f;

// The values the menus offer (also what clampSettings snaps to: the nearest one, ties upward).
inline constexpr int kMsaaChoices[] = {0, 2, 4, 8};
inline constexpr int kFpsCapChoices[] = {0, 60, 120, 144, 240};

// Snaps every field into its valid range or choice list (NaN becomes the default).
void clampSettings(Settings& s);

// JSON text of the settings (pretty printed, always the full set of keys).
std::string settingsToJson(const Settings& s);
// Reads the keys present in JSON text over `s` (the other fields keep their values), clamped. False, with `s` unchanged, when the
// text is not a JSON object. Used by settingsFromJson and by the scripted --do pref:JSON command.
bool mergeSettingsJson(Settings& s, const std::string& text);
// Reads settings from JSON text. Never throws; returns defaults for anything unreadable. `ok` (optional) is false when the text
// is not a JSON object at all (the result is then the defaults).
Settings settingsFromJson(const std::string& text, bool* ok = nullptr);

// "settings.json" in the working folder (next to the scenes/ and shaders/ folders).
std::string defaultSettingsPath();
// Loads the file. A missing file gives the defaults and true (first launch); an unreadable or corrupt one gives the defaults and
// false, with the reason in `note`.
bool loadSettings(const std::string& path, Settings& out, std::string& note);
// Writes the file. False with the reason in `error`.
bool saveSettings(const std::string& path, const Settings& s, std::string& error);

// Copies the display part of the settings into displayPrefs().
void applyDisplayPrefs(const Settings& s);

} // namespace app
} // namespace cosmos
