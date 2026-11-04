#pragma once
// The bottom bar (plan section 10.3) and the small windows hanging off its toolbar.
//
//   row 1:  [Start] [Pause] [Stop] [Save as scene] | time-rate slider with readout | [Resume warp] | date and clock | ... | toolbar
//   row 2:  one-line event / notes log (click it for the recent history)
//
// The bar owns no simulation state. Everything it reads or changes goes through RunHost, so the app (or the editor's
// RunControl, built separately) decides what Start / Pause / Stop / Save actually do. Toolbar toggles are plain flags in
// ToolbarState for whoever draws the panels they belong to (edit and add open the left panel; view, sim and stats open
// windows that live here).
#include <cstddef>
#include <deque>
#include <string>
#include <vector>
#include "imgui.h"
#include "ui/stats_panel.h"

namespace cosmos {
namespace ui {

enum class RunState {
    Stopped,   // the scene is being edited; nothing runs
    Running,
    Paused,    // started, clock held; editing is allowed (plan Q19)
};

// What the bottom bar needs from the application. All methods are called on the UI thread, once or a few times per frame.
class RunHost {
public:
    virtual ~RunHost() = default;

    // ---- run controls (plan Q17, Q19) ----
    virtual RunState runState() const = 0;
    virtual void start() = 0;                  // Stopped: snapshot the scene and run; Paused: continue
    virtual void pause() = 0;
    virtual void stop() = 0;                   // restore the snapshot taken at Start
    virtual void saveCurrentAsScene() = 0;     // keeps mid-run edits
    virtual bool canStart() const { return runState() != RunState::Running; }
    virtual bool canPause() const { return runState() == RunState::Running; }
    virtual bool canStop() const { return runState() != RunState::Stopped; }
    virtual bool canSaveCurrent() const { return true; }

    // ---- time rate (simulated seconds per real second) ----
    virtual double warp() const = 0;
    virtual void setWarp(double simSecondsPerSecond) = 0;
    virtual bool canResumeWarp() const = 0;    // true after an automatic slow-down
    virtual double resumeWarpValue() const { return warp(); }
    virtual void resumeWarp() = 0;

    // ---- readouts ----
    virtual double displayTime() const = 0;    // simulated seconds since the scene epoch (formatSimDate)
    virtual bool physicsBehind() const { return false; }

    // ---- simulation settings (the sim window) ----
    virtual int threadCount() const { return 1; }
    virtual void setThreadCount(int) {}
    virtual double controllerTick() const { return 0.1; }
    virtual void setControllerTick(double) {}
    virtual bool controllerTickEditable() const { return runState() != RunState::Running; }   // only while not running
    virtual bool infiniteFuel() const { return false; }
    virtual void setInfiniteFuel(bool) {}
    // "Slow down for any shuttle impact" (plan 8 item 4): without it only the followed or selected shuttle and the body pairs
    // slow the warp down.
    virtual bool slowDownAnyShuttle() const { return false; }
    virtual void setSlowDownAnyShuttle(bool) {}
};

// A short history of one-line messages (events, notes). Messages are pushed by the app; the bar shows the latest and
// lists the last ones when clicked.
class EventLog {
public:
    struct Entry {
        std::string text;
        double realTime = 0.0;   // seconds on a monotonic clock, at push time
    };
    static constexpr std::size_t kMaxEntries = 200;

    void push(const std::string& text);
    void clear() { entries_.clear(); }
    bool empty() const { return entries_.empty(); }
    const std::deque<Entry>& entries() const { return entries_; }
    const Entry* latest() const { return entries_.empty() ? nullptr : &entries_.back(); }
    double secondsSinceLatest() const;   // large when there is no entry

private:
    std::deque<Entry> entries_;
};

// Which toolbar windows are open. edit and add belong to the left panel (the editor reads and writes them).
struct ToolbarState {
    bool edit = false;
    bool add = false;
    bool view = false;
    bool sim = false;
    bool stats = false;
};

// Overlay switches of the view window (plan 9.5). The app copies them into the overlay builder and the renderer arrays each
// frame. overlaysAvailable false greys the checkboxes out (a host without overlays).
struct ViewOverlays {
    bool overlaysAvailable = true;
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
    float arrowLengthPx = 80.0f;   // the length of the longest arrow on screen
    int arrowScaling = 1;          // 0 linear, 1 sqrt, 2 log (app::ArrowScale order)
};

class BottomBar {
public:
    EventLog log;
    ToolbarState toolbar;
    ViewOverlays overlays;
    bool showLabels = true;          // "Names" in the view window
    StatsPanel stats;                // drawn by draw() while toolbar.stats is set; fed by the app

    // Draws the bar at the bottom edge of `display` (ImGui screen coordinates) and the windows the toolbar opened.
    // Call between UiContext::newFrame() and render().
    // `emphasis` (optional) is the font of the date readout.
    void draw(RunHost& host, ImVec2 display, ImFont* emphasis = nullptr);

    // Height of the bar in pixels as of the last draw() (0 before the first).
    float height() const { return height_; }
    // Screen rectangles (x0, y0, x1, y1) taken by the bar and its windows in the last draw(): keep-out areas for labels.
    const std::vector<ImVec4>& occupied() const { return occupied_; }

private:
    void drawViewWindow(ImVec2 display, float barTop);
    void drawSimWindow(RunHost& host, ImVec2 display, float barTop);
    void drawHistoryWindow(ImVec2 display, float barTop);
    void noteWindowRect();

    float height_ = 0.0f;
    bool showHistory_ = false;
    std::vector<ImVec4> occupied_;
};

} // namespace ui
} // namespace cosmos
