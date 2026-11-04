// AWL Cosmos application (plan M4, M6): the scene lives in a RunControl (editor model + snapshot + SimDriver: clock, physics
// ahead, interpolation), which the bottom bar's Start / Pause / Stop and the left panel drive; the camera moves and hands a
// camera-relative RenderFrame to the renderer. The ImGui UI (bottom bar, labels, stats) is
// drawn after the scene passes; the GLFW callbacks are installed BEFORE ImGui's so ImGui chains to them, and camera
// and keyboard input is gated on ImGui's WantCapture* flags (plan 10.4).
#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "app/bench_scene.h"
#include "app/bmp_writer.h"
#include "app/camera.h"
#include "app/explosions.h"
#include "app/format.h"
#include "app/overlay_fill.h"
#include "app/overlays.h"
#include "app/palette.h"
#include "app/perf_stats.h"
#include "app/presets.h"
#include "app/render_frame.h"
#include "app/run_control.h"
#include "app/slow_down.h"
#include "app/sim_driver.h"
#include "app/units.h"
#include "core/kinds.h"
#include "core/scene_json.h"
#include "core/threadpool.h"
#include "render/renderer.h"
#include "ui/bottom_bar.h"
#include "ui/labels.h"
#include "ui/left_panel.h"
#include "ui/theme.h"
#include "ui/ui_context.h"

using namespace cosmos;
using namespace cosmos::app;

namespace {

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;
constexpr double kPi = 3.14159265358979323846;
constexpr double kClickSlopPx = 4.0;        // a press-release that moved less than this is a click, not a drag
constexpr double kTitleInterval_s = 0.25;   // the window title (the M4 readout) is refreshed at this real-time interval
constexpr double kNoticeSeconds = 4.0;
constexpr double kStatsInterval_s = 0.1;    // conservation samples for the stats charts, real seconds

std::string lower(std::string s)
{
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

struct Options {
    int frames = -1;                  // --frames N: exit after N frames
    std::string screenshot;           // --screenshot path.bmp: write the last frame's back buffer
    bool vsync = true;                // --no-vsync turns it off
    double mpp = 0.0;                 // --mpp X: initial metres per pixel (0 = fit the whole system, or frame the --follow target)
    int width = kDefaultWidth, height = kDefaultHeight;   // --size W H: initial window size
    bool glDebug = false;             // --gl-debug: request a debug context and report GL errors (always on in Debug builds)
    std::string scene = "scenes/lucerna.json";   // --scene path (relative to the working folder, like shaders/) or a preset id (empty, lucerna)
    double warp = 1.0;                // --warp X: initial time warp
    std::string follow;               // --follow NAME: start following this body or shuttle
    double fixedDt = 0.0;             // --fixed-dt S: use S seconds per frame instead of the real time (repeatable screenshots)
    int threads = 0;                  // --threads N: shuttle-pass worker threads (0 = hardware concurrency, at most 16)
    bool ui = true;                   // --no-ui: draw only the scene (comparison runs)
    float uiScale = 0.0f;             // --ui-scale X: UI scale instead of the window's content scale
    std::string open;                 // --open view,sim,stats,edit,add: toolbar windows opened at start (nolabels: names off)
    struct Click { int frame = 0; double x = 0.0, y = 0.0; };   // --click X,Y@FRAME: cursor moves at FRAME, presses at +1, releases at +2
    std::vector<Click> clicks;
    int slowDownAt = -1;              // --slow-down-at N: trigger one automatic slow-down level at frame N (shows "Resume warp")
    std::string explode;              // --explode BODY@FRAME (developer aid, M7): a test explosion on BODY's surface at the camera centre
    int explodeAt = -1;
    std::string screenshotPrev;       // --screenshot-prev path.bmp: also write the frame before the last (needs --frames >= 2)
    // --do COMMAND@FRAME: scripted editor and run commands for verification (see runCommand): new:empty, add:star,
    // select:NAME, set:Mass=1 M_sun, orbit:PARENT,DISTANCE, start, pause, stop, save:PATH, load:PATH, delete, report, ...
    struct Command { int frame = 0; std::string text; };
    std::vector<Command> commands;
    // --type TEXT@FRAME types characters into the focused ImGui field; --enter FRAME presses Enter there (field editing)
    struct Typed { int frame = 0; std::string text; bool enter = false; };
    std::vector<Typed> typed;
    // Scripted input for verification without a human: --press KEY@FRAME (a key as in the help text: 0-9, A-Z, '.', ',',
    // '/', '-', '=', Space, Tab, Esc; a leading '+' adds Shift) and --scroll STEPS@FRAME (wheel notches at the cursor,
    // which starts at the window centre).
    struct Scripted { int frame = 0; bool scroll = false; int key = 0; int mods = 0; double steps = 0.0; };
    std::vector<Scripted> script;
    // --overlays LIST: switches overlays on at start, as if the View window's checkboxes were ticked, for screenshots. LIST is
    // comma separated: vel, acc, trails, orbits, path, wells, hill, soi, roche, atmo or all.
    std::string overlays;
    // --shot FILE.bmp@FRAME: also write the back buffer at the end of FRAME (several allowed; --screenshot is the last frame)
    struct Shot { int frame = 0; std::string path; };
    std::vector<Shot> shots;
    // Release benchmark (plan 14 "Performance (M8)"): --bench-shuttles N adds N deterministic shuttles to the loaded scene (see
    // app/bench_scene.h; combine with --do start@1, --no-vsync, --overlays all and a --follow / --mpp view), --bench-fuel KG
    // sets their tank, --bench-warmup N frames are left out of the statistics printed at the end (frame time percentiles, the
    // phases of a frame, the physics and overlay split), --perf-csv FILE writes the per-frame samples.
    int benchShuttles = 0;
    double benchFuel = kBenchDefaultFuel_kg;
    int benchWarmup = 120;
    std::string perfCsv;
    double benchPaceFps = 0.0;        // --bench-pace FPS: every frame advances the simulation by 1/FPS of real time (as at that frame rate), however fast frames are drawn
    std::string benchLabel;
    std::string benchControllers;     // --bench-controllers coast,prograde,retrograde,wander: the autopilots the benchmark shuttles cycle through
};

// Parses "KEY@FRAME" / "STEPS@FRAME" for --press / --scroll. Returns false on a malformed or unknown key.
bool parseScripted(const std::string& text, bool scroll, Options::Scripted& out)
{
    const std::size_t at = text.rfind('@');
    out.scroll = scroll;
    out.frame = at == std::string::npos ? 0 : std::atoi(text.c_str() + at + 1);
    std::string head = text.substr(0, at);
    if (scroll) { out.steps = std::atof(head.c_str()); return true; }
    if (head.size() > 1 && head[0] == '+') { out.mods = GLFW_MOD_SHIFT; head.erase(0, 1); }
    const std::string h = lower(head);
    if (h == "space") out.key = GLFW_KEY_SPACE;
    else if (h == "tab") out.key = GLFW_KEY_TAB;
    else if (h == "esc") out.key = GLFW_KEY_ESCAPE;
    else if (h.size() == 1) out.key = std::toupper(static_cast<unsigned char>(h[0]));   // GLFW key codes are the upper-case ASCII
    else return false;
    return true;
}

bool parseArgs(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) o.frames = std::atoi(argv[++i]);
        else if (a == "--screenshot" && i + 1 < argc) o.screenshot = argv[++i];
        else if (a == "--mpp" && i + 1 < argc) o.mpp = std::atof(argv[++i]);
        else if (a == "--size" && i + 2 < argc) { o.width = std::atoi(argv[++i]); o.height = std::atoi(argv[++i]); }
        else if (a == "--scene" && i + 1 < argc) o.scene = argv[++i];
        else if (a == "--warp" && i + 1 < argc) o.warp = std::atof(argv[++i]);
        else if (a == "--follow" && i + 1 < argc) o.follow = argv[++i];
        else if (a == "--fixed-dt" && i + 1 < argc) o.fixedDt = std::atof(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) o.threads = std::atoi(argv[++i]);
        else if (a == "--ui-scale" && i + 1 < argc) o.uiScale = static_cast<float>(std::atof(argv[++i]));
        else if (a == "--open" && i + 1 < argc) o.open = argv[++i];
        else if (a == "--screenshot-prev" && i + 1 < argc) o.screenshotPrev = argv[++i];
        else if (a == "--click" && i + 1 < argc) {
            Options::Click c;
            const std::string v = argv[++i];
            const std::size_t comma = v.find(','), at = v.rfind('@');
            if (comma == std::string::npos || at == std::string::npos || at < comma) { std::cerr << "Bad value for --click: " << v << "\n"; return false; }
            c.x = std::atof(v.c_str());
            c.y = std::atof(v.c_str() + comma + 1);
            c.frame = std::atoi(v.c_str() + at + 1);
            o.clicks.push_back(c);
        }
        else if (a == "--slow-down-at" && i + 1 < argc) o.slowDownAt = std::atoi(argv[++i]);
        else if (a == "--explode" && i + 1 < argc) {
            const std::string v = argv[++i];
            const std::size_t at = v.rfind('@');
            o.explode = v.substr(0, at);
            o.explodeAt = at == std::string::npos ? 1 : std::atoi(v.c_str() + at + 1);
        }
        else if (a == "--do" && i + 1 < argc) {
            const std::string v = argv[++i];
            const std::size_t at = v.rfind('@');
            if (at == std::string::npos) { std::cerr << "Bad value for --do (expected COMMAND@FRAME): " << v << "\n"; return false; }
            o.commands.push_back({std::atoi(v.c_str() + at + 1), v.substr(0, at)});
        }
        else if (a == "--type" && i + 1 < argc) {
            const std::string v = argv[++i];
            const std::size_t at = v.rfind('@');
            if (at == std::string::npos) { std::cerr << "Bad value for --type (expected TEXT@FRAME): " << v << "\n"; return false; }
            o.typed.push_back({std::atoi(v.c_str() + at + 1), v.substr(0, at), false});
        }
        else if (a == "--enter" && i + 1 < argc) o.typed.push_back({std::atoi(argv[++i]), std::string(), true});
        else if (a == "--no-ui") o.ui = false;
        else if ((a == "--press" || a == "--scroll") && i + 1 < argc) {
            Options::Scripted sc;
            if (!parseScripted(argv[++i], a == "--scroll", sc)) { std::cerr << "Bad value for " << a << ": " << argv[i] << "\n"; return false; }
            o.script.push_back(sc);
        }
        else if (a == "--no-vsync") o.vsync = false;
        else if (a == "--bench-shuttles" && i + 1 < argc) o.benchShuttles = std::atoi(argv[++i]);
        else if (a == "--bench-fuel" && i + 1 < argc) o.benchFuel = std::atof(argv[++i]);
        else if (a == "--bench-warmup" && i + 1 < argc) o.benchWarmup = std::atoi(argv[++i]);
        else if (a == "--perf-csv" && i + 1 < argc) o.perfCsv = argv[++i];
        else if (a == "--bench-controllers" && i + 1 < argc) o.benchControllers = argv[++i];
        else if (a == "--bench-label" && i + 1 < argc) o.benchLabel = argv[++i];
        else if (a == "--bench-pace" && i + 1 < argc) o.benchPaceFps = std::atof(argv[++i]);
        else if (a == "--gl-debug") o.glDebug = true;
        else if (a == "--overlays" && i + 1 < argc) o.overlays = argv[++i];
        else if (a == "--shot" && i + 1 < argc) {
            const std::string v = argv[++i];
            const std::size_t at = v.rfind('@');
            if (at == std::string::npos) { std::cerr << "Bad value for --shot (expected FILE@FRAME): " << v << "\n"; return false; }
            o.shots.push_back({std::atoi(v.c_str() + at + 1), v.substr(0, at)});
        }
        else {
            std::cerr << "Unknown or incomplete option: " << a << "\n"
                      << "Usage: AWL_Cosmos [--scene file.json] [--warp X] [--follow NAME] [--mpp X] [--size W H] [--frames N]\n"
                      << "                  [--screenshot file.bmp] [--screenshot-prev file.bmp] [--fixed-dt S] [--threads N]\n"
                      << "                  [--no-vsync] [--gl-debug] [--no-ui] [--ui-scale X] [--open view,sim,stats,log,edit,add]\n"
                      << "                  [--click X,Y@FRAME]... [--slow-down-at FRAME] [--do COMMAND@FRAME]...\n"
                      << "                  [--type TEXT@FRAME]... [--enter FRAME]...\n"
                      << "                  [--press KEY@FRAME]... [--scroll STEPS@FRAME]...\n"
                      << "                  [--overlays vel,acc,trails,orbits,path,wells,hill,soi,roche,atmo|all] [--shot FILE.bmp@FRAME]...\n"
                      << "                  [--bench-shuttles N] [--bench-fuel KG] [--bench-warmup FRAMES] [--bench-label TEXT]"
                      << " [--bench-controllers LIST] [--bench-pace FPS] [--perf-csv FILE]\n";
            return false;
        }
    }
    return true;
}

// Per-kind look (the old kindProps table, extended to the plan 12.1 kinds): stars and neutron stars glow and light the
// others, a black hole is a dark disc with a ring, everything else is lit by the lights plus a faint ambient.
struct KindLook {
    bool emitsLight;
    bool receivesLight;
    float ambient;
};

KindLook kindLook(BodyKind k)
{
    switch (k) {
    case BodyKind::Star:        return {true, false, 0.00f};
    case BodyKind::NeutronStar: return {true, false, 0.00f};
    case BodyKind::BlackHole:   return {false, false, 0.00f};
    case BodyKind::Planet:      return {false, true, 0.04f};
    case BodyKind::GasGiant:    return {false, true, 0.04f};
    case BodyKind::DwarfPlanet: return {false, true, 0.03f};
    case BodyKind::Moon:        return {false, true, 0.02f};
    case BodyKind::Asteroid:    return {false, true, 0.02f};
    }
    return {false, true, 0.04f};
}

constexpr float kStarLightIntensity = 1.3f;

// ---------------------------------------------------------------------------------------------------------------
// The application state shared by the GLFW callbacks and the frame loop.
struct App {
    explicit App(const RunOptions& runOptions) : rc(runOptions), panel(rc) {}

    GLFWwindow* window = nullptr;
    RunControl rc;                   // editor model, snapshot and the SimDriver (a new driver object after some operations)
    std::uint64_t seenGeneration = 0;   // rc.generation() the lookups below were built for
    Camera camera;
    CameraSources src;
    ShuttlePalette palette{2024};
    ui::UiContext ui;                // ImGui context and backends (initialised after the callbacks below are installed)
    ui::BottomBar bar;               // bottom bar, its windows, the event log and the stats history
    ui::LeftPanel panel;             // the scene editor (list, fields, scene menu, add palette)
    ui::LabelOptions labelOptions;
    SlowDown slowDown;               // warp step-down for approaching impacts (M7, plan 8)
    ExplosionSystem explosions;      // explosion registry; fills RenderFrame::explosions (M7, plan 9.7)
    ThreadPool workers;              // frame-building workers (overlay geometry, line upload); the main thread takes part
    OverlayBuilder overlays;         // arrows, trails, osculating orbits, predicted path (M7, plan 9.5)
    RadiiCache radiiCache;           // throttled Hill / SOI / Roche radii for the circles
    double lastStatSimTime = -1.0e300;   // physics time of the last conservation sample
    double wellsEtcMs = 0.0;         // benchmark: wall time of the wells / atmospheres / circles part of fillOverlays

    int winW = 0, winH = 0;          // window size in screen coordinates (cursor units)
    int fbW = 0, fbH = 0;            // framebuffer size in pixels
    bool dirty = true;               // framebuffer size changed

    // Settings of the sim window. They belong to the user, not to a driver object: re-applied after every rebuild.
    int threads = 1;
    double controllerTick = 0.1;
    bool infiniteFuel = false;

    // A "frame this object" request from the panel, honoured once the object exists in the driver's state
    struct PendingFrame { TargetKind kind = TargetKind::None; std::uint64_t id = 0; int tries = 0; };
    PendingFrame pendingFrame;

    // Lookups rebuilt from rc.driver().state() every frame (ids are unique across bodies and shuttles).
    std::unordered_map<std::uint64_t, std::size_t> bodyIndex, shuttleIndex;
    std::unordered_map<std::uint64_t, std::uint64_t> survivorOf;   // absorbed body -> survivor, from due Merge events
    std::unordered_map<std::uint64_t, std::string> names;          // every body and shuttle ever seen

    // Mouse
    double cursorX = 0.0, cursorY = 0.0;      // window coordinates
    bool leftDown = false, rightDown = false;
    double dragPx = 0.0;                      // distance moved since the press (framebuffer pixels)
    bool havePointer = false;

    // Reference frame bookkeeping and the notice line of the title
    std::uint64_t lastFollowedBody = 0;
    std::string notice;
    double noticeUntil = 0.0;
    bool quit = false;
    // Bodies of the keys 1-9: the nine lowest IDs of the starting scene, fixed once (a merge must not renumber the keys)
    std::array<std::uint64_t, 9> numberKeys{};

    Viewport viewport() const { return {fbW, fbH}; }
    double pixelScale() const { return winW > 0 ? static_cast<double>(fbW) / winW : 1.0; }   // window units -> pixels

    const DisplayState& state() const { return rc.driver().state(); }
    void say(const std::string& s)
    {
        notice = s;
        noticeUntil = glfwGetTime() + kNoticeSeconds;
        bar.log.push(s);
        std::cout << (frameNo >= 0 ? "(f" + std::to_string(frameNo) + ") " : std::string()) << s << "\n";   // frame: for --shot / --do scripting
    }
    long long frameNo = -1;   // the frame being built (1-based), -1 before the loop
};

std::string nameOf(const App& app, std::uint64_t id)
{
    auto it = app.names.find(id);
    return it != app.names.end() && !it->second.empty() ? it->second : "#" + std::to_string(id);
}

void rebuildLookups(App& app)
{
    const DisplayState& st = app.state();
    app.bodyIndex.clear();
    app.shuttleIndex.clear();
    for (std::size_t i = 0; i < st.bodies.size(); ++i) {
        app.bodyIndex[st.bodies[i].id] = i;
        app.names[st.bodies[i].id] = st.bodies[i].name;
    }
    const Shuttles& sh = app.rc.driver().sim().shuttles();
    for (std::size_t i = 0; i < st.shuttles.size(); ++i) {
        const DisplayShuttle& s = st.shuttles[i];
        app.shuttleIndex[s.id] = i;
        if (s.index < sh.name.size()) app.names[s.id] = sh.name[s.index];
    }
}

// Where objects are for the camera (plan 9.3): the display state at the display time, so what the camera follows is
// exactly what is drawn (no jitter); any other time falls back to the dense output of the latest body step.
void buildSources(App& app)
{
    App* a = &app;
    CameraSources& s = app.src;
    s.bodyPosition = [a](std::uint64_t id, double t, Vec3& out) {
        if (t == a->state().t) {
            auto it = a->bodyIndex.find(id);
            if (it == a->bodyIndex.end()) return false;
            out = a->state().bodies[it->second].r;
            return true;
        }
        return a->rc.driver().sim().bodyPositionAt(id, t, out);
    };
    s.bodyVelocity = [a](std::uint64_t id, double t, Vec3& out) {
        if (t == a->state().t) {
            auto it = a->bodyIndex.find(id);
            if (it == a->bodyIndex.end()) return false;
            out = a->state().bodies[it->second].v;
            return true;
        }
        return a->rc.driver().sim().bodyVelocityAt(id, t, out);
    };
    s.shuttlePosition = [a](std::uint64_t id, double t, Vec3& out) {
        auto it = a->shuttleIndex.find(id);
        if (it == a->shuttleIndex.end()) return false;
        const DisplayShuttle& sh = a->state().shuttles[it->second];
        if (t == a->state().t) { out = sh.r; return true; }
        out = a->rc.driver().sim().shuttlePositionAt(sh.index, t);
        return true;
    };
    // System barycentre of the bodies at the display time (mu is proportional to mass).
    auto barycentre = [a](bool velocity, Vec3& out) {
        double m = 0.0, x = 0.0, y = 0.0, z = 0.0;
        for (const DisplayBody& b : a->state().bodies) {
            const Vec3& p = velocity ? b.v : b.r;
            m += b.mu; x += b.mu * p.x; y += b.mu * p.y; z += b.mu * p.z;
        }
        if (!(m > 0.0)) return false;
        out = {x / m, y / m, z / m};
        return true;
    };
    s.barycentre = [a, barycentre](double t, Vec3& out) { return t == a->state().t && barycentre(false, out); };
    s.barycentreVelocity = [a, barycentre](double t, Vec3& out) { return t == a->state().t && barycentre(true, out); };
    s.mergeSurvivor = [a](std::uint64_t id) {
        auto it = a->survivorOf.find(id);
        return it != a->survivorOf.end() ? it->second : std::uint64_t(0);
    };
}

// ---------------------------------------------------------------------------------------------------------------
// Framing

// Metres per pixel that frames a target: for a body, its satellites (the bodies inside its Hill sphere about the body
// that pulls it hardest) fit the window; for a shuttle, the planet it orbits and the shuttle fit. A body without
// satellites fills about a quarter of the window.
double framingMpp(const App& app, TargetKind kind, std::uint64_t id)
{
    const DisplayState& st = app.state();
    const double half = 0.5 * std::max(1, std::min(app.fbW, app.fbH));
    auto dist = [](const Vec3& a, const Vec3& b) { return std::hypot(a.x - b.x, a.y - b.y); };
    // The body that pulls position p hardest (mu / d^2), other than `skip`, among bodies heavier than minMu.
    auto attractor = [&](const Vec3& p, std::uint64_t skip, double minMu) -> const DisplayBody* {
        const DisplayBody* best = nullptr;
        double bestPull = 0.0;
        for (const DisplayBody& b : st.bodies) {
            if (b.id == skip || b.absorbed || !(b.mu > minMu)) continue;
            const double d = std::max(dist(p, b.r), 1.0);
            const double pull = b.mu / (d * d);
            if (pull > bestPull) { bestPull = pull; best = &b; }
        }
        return best;
    };
    double extent = 0.0;
    if (kind == TargetKind::Body) {
        auto it = app.bodyIndex.find(id);
        if (it == app.bodyIndex.end()) return 0.0;
        const DisplayBody& b = st.bodies[it->second];
        // Hill radius of b about its parent (infinite for a body nothing heavier pulls on): d * (mu / (3 mu_parent))^(1/3)
        double hill = std::numeric_limits<double>::infinity();
        if (const DisplayBody* parent = attractor(b.r, b.id, b.mu))
            hill = dist(b.r, parent->r) * std::cbrt(b.mu / (3.0 * parent->mu));
        for (const DisplayBody& o : st.bodies) {
            if (o.id == b.id || o.mu >= b.mu || o.absorbed) continue;
            const double d = dist(o.r, b.r);
            if (d < hill) extent = std::max(extent, d);
        }
        if (extent <= 0.0) extent = 4.0 * std::max(b.radius, 1.0);
    } else if (kind == TargetKind::Shuttle) {
        auto it = app.shuttleIndex.find(id);
        if (it == app.shuttleIndex.end()) return 0.0;
        const DisplayShuttle& s = st.shuttles[it->second];
        const DisplayBody* a = attractor(s.r, 0, 0.0);
        extent = a ? dist(s.r, a->r) : 100.0;
        extent = std::max(extent, 20.0);
    }
    return 1.3 * extent / half;
}

// Whole system in the window: the farthest body (plus its radius) from the origin fits half the window.
double fitSystemMpp(const App& app)
{
    double far = 1.0;
    for (const DisplayBody& b : app.state().bodies) far = std::max(far, std::hypot(b.r.x, b.r.y) + b.radius);
    const double half = 0.5 * std::max(1, std::min(app.fbW, app.fbH));
    return 1.15 * far / half;
}

void followTarget(App& app, TargetKind kind, std::uint64_t id, bool frame)
{
    app.camera.follow(kind, id);
    if (kind == TargetKind::Body) app.lastFollowedBody = id;
    if (frame) {
        const double mpp = framingMpp(app, kind, id);
        if (mpp > 0.0) app.camera.setMetresPerPixel(mpp);
    }
    app.say("Following " + nameOf(app, id));
}

// F / Shift+F / Tab: next or previous target in the order bodies (ID order) then shuttles (ID order).
void cycleFollow(App& app, int dir)
{
    struct T { TargetKind kind; std::uint64_t id; };
    std::vector<T> list;
    for (const DisplayBody& b : app.state().bodies) if (!b.absorbed) list.push_back({TargetKind::Body, b.id});
    for (const DisplayShuttle& s : app.state().shuttles) if (!s.dying) list.push_back({TargetKind::Shuttle, s.id});
    if (list.empty()) return;
    const Follow cur = app.camera.following();
    int idx = -1;
    for (std::size_t i = 0; i < list.size(); ++i)
        if (cur.active() && list[i].id == cur.id) idx = static_cast<int>(i);
    const int n = static_cast<int>(list.size());
    const int next = idx < 0 ? (dir > 0 ? 0 : n - 1) : ((idx + dir) % n + n) % n;
    followTarget(app, list[static_cast<std::size_t>(next)].kind, list[static_cast<std::size_t>(next)].id, true);
}

void followByName(App& app, const std::string& name, bool frame)
{
    const std::string want = lower(name);
    for (const DisplayBody& b : app.state().bodies)
        if (lower(b.name) == want) { followTarget(app, TargetKind::Body, b.id, frame); return; }
    for (const DisplayShuttle& s : app.state().shuttles)
        if (lower(nameOf(app, s.id)) == want) { followTarget(app, TargetKind::Shuttle, s.id, frame); return; }
    app.say("No body or shuttle named '" + name + "'");
}

void setFrameMode(App& app, FrameMode mode)
{
    std::uint64_t body = 0;
    if (mode == FrameMode::FollowBody) {
        const Follow f = app.camera.following();
        body = (f.kind == TargetKind::Body) ? f.id : app.lastFollowedBody;
        if (body == 0) { app.say("Reference frame 'follow body' needs a followed body (press 1-9 or F first)"); return; }
    }
    if (!app.camera.setFrame(mode, body, app.state().t, app.src)) { app.say("Reference frame change refused"); return; }
    app.say(mode == FrameMode::Inertial ? "Frame: inertial"
            : mode == FrameMode::Barycentre ? "Frame: barycentre"
                                            : "Frame: relative to " + nameOf(app, body));
}

void resetView(App& app)
{
    app.camera.stopFollowing();
    app.camera.setCentreWorld({0.0, 0.0, 0.0});
    app.camera.setMetresPerPixel(fitSystemMpp(app));
}

// A click: follow the nearest object within the pick radius (plan 9.3, picking against camera-relative positions).
void clickPick(App& app)
{
    std::vector<PickCandidate> cands;
    for (const DisplayBody& b : app.state().bodies) {
        const Camera::Rel r = app.camera.toCameraRelative(b.r);
        cands.push_back({TargetKind::Body, b.id, r.x, r.y, static_cast<float>(b.radius)});
    }
    for (const DisplayShuttle& s : app.state().shuttles) {
        const Camera::Rel r = app.camera.toCameraRelative(s.r);
        cands.push_back({TargetKind::Shuttle, s.id, r.x, r.y, 1.0f});
    }
    const double k = app.pixelScale();
    double rx = 0.0, ry = 0.0;
    app.camera.windowToRelative(app.cursorX * k, app.cursorY * k, app.viewport(), rx, ry);
    const PickResult pr = pickNearest(cands, rx, ry, app.camera.metresPerPixel());
    if (pr.hit) {
        followTarget(app, pr.kind, pr.id, false);
        app.panel.selectObject(pr.id);   // the viewport pick selects it in the editor too (the list scrolls to it)
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Run control glue (M6): what Start / Pause / Stop do beyond RunControl itself

// Re-applies the sim window's settings to the current driver's simulation (a rebuild starts from the scene's settings).
void applyPrefs(App& app)
{
    Simulation& sim = app.rc.driver().sim();
    if (sim.threadCount() != app.threads) sim.setThreadCount(app.threads);
    if (app.workers.threads() != app.threads) app.workers.resize(app.threads);
    if (sim.controllerTick() != app.controllerTick) sim.setControllerTick(app.controllerTick);
    if (sim.infiniteFuel() != app.infiniteFuel) sim.setInfiniteFuel(app.infiniteFuel);
}

void logEvent(App& app, const Event& e);
void resetStats(App& app);

// RunControl replaces its driver on some operations (start from Stopped, stop, load, an edit while paused): whatever was cached
// from the old one is dropped, the settings are re-applied and the stats start over (the new simulation has its own
// reference energy and momentum). Call once per frame after RunControl::advance.
void syncGeneration(App& app)
{
    if (app.rc.generation() == app.seenGeneration) return;
    app.seenGeneration = app.rc.generation();
    // Events the old simulation had computed past the display time and an edit cut off: log them while the names of the old
    // simulation are still known, and keep the merges so a followed body that was absorbed leads to its survivor.
    const std::vector<Event> cutOff = app.rc.takeFlushedEvents();
    for (const Event& e : cutOff) logEvent(app, e);
    app.names.clear();
    app.survivorOf.clear();
    app.explosions.clear();   // the old simulation's events and bodies are gone
    app.overlays.reset();     // trails, orbits and the prediction belong to the old simulation (Stop, Start, load, edit rebuild)
    app.radiiCache.reset();
    for (const Event& e : cutOff)
        if (e.type == EventType::Merge) app.survivorOf[e.other] = e.id;
    applyPrefs(app);
    resetStats(app);
}

// Keys 1-9: the nine lowest body IDs of the scene as it starts (a merge must not renumber them)
void fillNumberKeys(App& app)
{
    app.numberKeys = {};
    std::vector<std::uint64_t> ids;
    for (const DisplayBody& b : app.state().bodies) ids.push_back(b.id);
    std::sort(ids.begin(), ids.end());
    for (std::size_t k = 0; k < app.numberKeys.size() && k < ids.size(); ++k) app.numberKeys[k] = ids[k];
}

void resetStats(App& app)
{
    app.bar.stats.reset();
    app.lastStatSimTime = -1.0e300;
}

void runStart(App& app)
{
    const bool fromStopped = app.rc.state() == RunState::Stopped;
    const app::EditResult r = app.rc.start();
    if (!r) { app.say(r.error); return; }
    if (fromStopped) {
        resetStats(app);
        syncGeneration(app);
        rebuildLookups(app);
        fillNumberKeys(app);
        app.say("Started: the scene was saved as the snapshot Stop returns to");
    } else {
        app.say("Resumed");
    }
    syncGeneration(app);
    rebuildLookups(app);
}

void runPause(App& app)
{
    const app::EditResult r = app.rc.pause();
    if (!r) { app.say(r.error); return; }
    app.say("Paused: edit freely, Start continues");
    syncGeneration(app);
    rebuildLookups(app);
}

void runStop(App& app)
{
    const app::EditResult r = app.rc.stop();
    if (!r) { app.say(r.error); return; }
    resetStats(app);
    app.say("Stopped: the scene is back as it was at Start");
    syncGeneration(app);
    rebuildLookups(app);
}

// A new document (preset or file) was loaded into the RunControl: a clean view, keys, stats and settings.
void onSceneReplaced(App& app)
{
    syncGeneration(app);
    rebuildLookups(app);
    fillNumberKeys(app);
    const SceneSettings& settings = app.rc.editor().scene().settings;
    app.infiniteFuel = settings.infiniteFuel;
    app.controllerTick = settings.controllerTick_s;
    applyPrefs(app);
    app.lastFollowedBody = 0;
    app.pendingFrame = {};
    app.camera.stopFollowing();
    app.camera.setFrame(FrameMode::Inertial, 0, app.state().t, app.src);
    app.camera.setCentreWorld({0.0, 0.0, 0.0});
    app.camera.setMetresPerPixel(app.state().bodies.empty() ? 1.0e7 : fitSystemMpp(app));
    resetStats(app);
}

// ---------------------------------------------------------------------------------------------------------------
// GLFW callbacks

void framebufferSizeCallback(GLFWwindow* w, int width, int height)
{
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
    app->fbW = width;
    app->fbH = height;
    glfwGetWindowSize(w, &app->winW, &app->winH);
    app->dirty = true;
}

void scrollCallback(GLFWwindow* w, double, double dy)
{
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
    if (app->fbW <= 0 || dy == 0.0 || app->ui.wantsMouse()) return;   // the wheel scrolls a panel, not the camera
    const double k = app->pixelScale();
    app->camera.zoomAtWindow(dy, app->cursorX * k, app->cursorY * k, app->viewport());
}

void cursorPosCallback(GLFWwindow* w, double x, double y)
{
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
    if (app->havePointer && (app->leftDown || app->rightDown)) {
        const double k = app->pixelScale();
        const double dx = (x - app->cursorX) * k, dy = (y - app->cursorY) * k;
        app->camera.panByWindowPixels(dx, dy);
        app->dragPx += std::hypot(dx, dy);
    }
    app->cursorX = x;
    app->cursorY = y;
    app->havePointer = true;
}

void mouseButtonCallback(GLFWwindow* w, int button, int action, int)
{
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
    if (button != GLFW_MOUSE_BUTTON_LEFT && button != GLFW_MOUSE_BUTTON_RIGHT) return;
    bool& down = button == GLFW_MOUSE_BUTTON_LEFT ? app->leftDown : app->rightDown;
    if (action == GLFW_PRESS) {
        if (app->ui.wantsMouse()) return;   // the press belongs to a panel: no pan, no pick
        if (!app->leftDown && !app->rightDown) app->dragPx = 0.0;
        down = true;
    } else if (action == GLFW_RELEASE) {
        const bool ours = down;             // the press went to the camera
        down = false;
        if (ours && button == GLFW_MOUSE_BUTTON_LEFT && app->dragPx < kClickSlopPx && !app->ui.wantsMouse()) clickPick(*app);
    }
}

void keyCallback(GLFWwindow* w, int key, int, int action, int mods)
{
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
    if (action == GLFW_RELEASE || app->ui.wantsKeyboard()) return;   // typing into a field: no shortcuts
    SimDriver& d = app->rc.driver();
    const bool repeat = action == GLFW_REPEAT;

    // Zoom keys may repeat; the rest act once per press.
    if (key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD) { app->camera.zoomAtCentred(1.0, 0.0, 0.0); return; }
    if (key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT) { app->camera.zoomAtCentred(-1.0, 0.0, 0.0); return; }
    if (key == GLFW_KEY_PAGE_UP) { app->camera.zoomAtCentred(4.0, 0.0, 0.0); return; }
    if (key == GLFW_KEY_PAGE_DOWN) { app->camera.zoomAtCentred(-4.0, 0.0, 0.0); return; }
    if (repeat) return;

    switch (key) {
    case GLFW_KEY_PERIOD: d.warp().stepUp(); break;                 // '.' warp up the ladder
    case GLFW_KEY_COMMA: d.warp().stepDown(); break;                // ',' warp down
    case GLFW_KEY_SLASH: d.warp().set(1.0); break;                  // '/' back to real time
    case GLFW_KEY_SPACE:   // run <-> pause; from Stopped it starts
        if (app->rc.running()) runPause(*app); else runStart(*app);
        break;
    case GLFW_KEY_F: cycleFollow(*app, (mods & GLFW_MOD_SHIFT) ? -1 : 1); break;
    case GLFW_KEY_TAB: cycleFollow(*app, (mods & GLFW_MOD_SHIFT) ? -1 : 1); break;
    case GLFW_KEY_0: setFrameMode(*app, FrameMode::Inertial); break;
    case GLFW_KEY_B: setFrameMode(*app, FrameMode::Barycentre); break;
    case GLFW_KEY_R: setFrameMode(*app, FrameMode::FollowBody); break;
    case GLFW_KEY_H:
    case GLFW_KEY_HOME: resetView(*app); break;
    case GLFW_KEY_ESCAPE:   // only stops following; quitting is the window close button or Ctrl+Q
        if (app->camera.following().active()) { app->camera.stopFollowing(); app->say("Stopped following"); }
        break;
    case GLFW_KEY_Q:
        if (mods & GLFW_MOD_CONTROL) app->quit = true;
        break;
    default:
        // 1-9: follow the n-th body of the starting scene (ID order); in Lucerna 4 is Domus. The table is fixed at start,
        // so a merge never renumbers the keys; a body that has merged away leads to the body that absorbed it.
        if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
            std::uint64_t id = app->numberKeys[static_cast<std::size_t>(key - GLFW_KEY_1)];
            for (int hops = 0; id != 0 && hops < 16; ++hops) {
                if (app->bodyIndex.count(id)) { followTarget(*app, TargetKind::Body, id, true); break; }
                auto it = app->survivorOf.find(id);
                if (it == app->survivorOf.end()) { app->say(nameOf(*app, id) + " is gone"); break; }
                id = it->second;
            }
        }
        break;
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Frame building (plan 9.3: subtract the camera in double, cast to float afterwards)

void fillFrame(const App& app, RenderFrame& frame)
{
    const DisplayState& st = app.state();
    const Camera& cam = app.camera;
    frame.clearObjects();
    frame.viewportWidth = app.fbW;
    frame.viewportHeight = app.fbH;
    frame.metresPerPixel = static_cast<float>(cam.metresPerPixel());
    // Minimum hull size (plan 9.3): 8 px close up, shrinking to a body-sized 3 px marker from 1 km/px to 1000 km/px, so
    // a planet's whole fleet does not hide the planet when the system is in view.
    {
        const double lg = std::log10(cam.metresPerPixel());
        const double u = std::min(1.0, std::max(0.0, (lg - 3.0) / 3.0));
        frame.shuttleMinPixels = static_cast<float>(8.0 + (kMarkerDiameterPx - 8.0) * u);
    }

    for (const DisplayBody& b : st.bodies) {      // absorbed bodies are drawn too, until their merge time
        const KindLook look = kindLook(b.kind);
        const Camera::Rel r = cam.toCameraRelative(b.r);
        RenderBody rb;
        rb.x = r.x;
        rb.y = r.y;
        rb.radius = static_cast<float>(b.radius);
        rb.colour = {b.colour[0], b.colour[1], b.colour[2]};
        rb.ambient = look.ambient;
        rb.receivesLight = look.receivesLight;
        rb.emissive = !look.receivesLight;
        rb.isBlackHole = b.kind == BodyKind::BlackHole;
        frame.bodies.push_back(rb);
        if (look.emitsLight) {
            RenderLight l;
            l.x = r.x;
            l.y = r.y;
            l.colour = {0.5f + 0.5f * b.colour[0], 0.5f + 0.5f * b.colour[1], 0.5f + 0.5f * b.colour[2]};   // tinted toward white
            l.intensity = kStarLightIntensity;
            frame.lights.push_back(l);
        }
    }
    for (const DisplayShuttle& s : st.shuttles) {   // dying shuttles are drawn until their contact time
        const Camera::Rel r = cam.toCameraRelative(s.r);
        RenderShuttle rs;
        rs.x = r.x;
        rs.y = r.y;
        rs.heading = static_cast<float>(std::remainder(s.heading, 2.0 * kPi));   // wrapped first: float keeps its precision
        const auto cols = app.palette.coloursFor(s.id);
        for (std::size_t k = 0; k < 3; ++k) rs.sector[k] = {cols[k].r, cols[k].g, cols[k].b};
        rs.throttle = static_cast<float>(std::min(1.0, std::max(0.0, s.throttle)));
        rs.flickerPhase = static_cast<float>(std::fmod(static_cast<double>(s.id) * 1.7, 2.0 * kPi));
        frame.shuttles.push_back(rs);
    }
}

// The View window's switches into the overlay builder (called every frame; cheap).
void applyOverlaySettings(App& app)
{
    const ui::ViewOverlays& v = app.bar.overlays;
    OverlaySettings& s = app.overlays.settings;
    s.velocityArrows = v.velocityArrows;
    s.accelerationArrows = v.accelerationArrows;
    s.trails = v.trails;
    s.osculatingOrbits = v.osculatingOrbits;
    s.predictedPath = v.predictedPath;
    s.arrowLengthPx = static_cast<double>(v.arrowLengthPx);
    s.arrowMode = v.arrowScaling <= 0 ? ArrowScale::Linear : (v.arrowScaling == 1 ? ArrowScale::Sqrt : ArrowScale::Log);
    // The predicted path follows the followed object; with nothing followed, the selected object of the editor.
    TargetKind kind = TargetKind::None;
    std::uint64_t id = 0;
    if (app.camera.following().kind == TargetKind::None && app.rc.editor().selected() != 0) {
        id = app.rc.editor().selected();
        if (app.shuttleIndex.count(id)) kind = TargetKind::Shuttle;
        else if (app.bodyIndex.count(id)) kind = TargetKind::Body;
        else id = 0;
    }
    app.overlays.setPredictionTarget(kind, id);
}

// Plan 9.4 steps 2-5 and 9: everything the overlays add to the frame, after fillFrame pushed the bodies (the well, atmosphere
// and circle arrays read the camera-relative body positions from frame.bodies, in the order of the display bodies).
void fillOverlays(App& app, RenderFrame& frame)
{
    applyOverlaySettings(app);
    const double tStart = glfwGetTime();
    const ui::ViewOverlays& v = app.bar.overlays;
    const std::vector<DisplayBody>& bodies = app.state().bodies;
    if (v.gravityRings) appendWells(frame, bodies);
    if (v.atmospheres) appendAtmospheres(frame, bodies);
    if (v.hill || v.soi || v.roche) {
        CircleSet which;
        which.hill = v.hill;
        which.soi = v.soi;
        which.roche = v.roche;
        // The O(n^3) hierarchy is recomputed at 4 Hz, not every frame (RadiiCache).
        appendCircles(frame, app.radiiCache.get(bodies, app.rc.generation(), frame.time), which);
    }
    app.wellsEtcMs = (glfwGetTime() - tStart) * 1000.0;
    app.overlays.build(app.rc.driver(), app.rc.generation(), OverlayView::fromCamera(app.camera), frame);
}

// --overlays LIST (screenshots): ticks the View window's checkboxes.
void applyOverlayFlag(ui::ViewOverlays& v, const std::string& list)
{
    auto has = [&list](const char* w) { return list.find(w) != std::string::npos; };
    const bool all = has("all");
    v.velocityArrows = all || has("vel");
    v.accelerationArrows = all || has("acc");
    v.trails = all || has("trails");
    v.osculatingOrbits = all || has("orbits");
    v.predictedPath = all || has("path");
    v.gravityRings = all || has("wells");
    v.hill = all || has("hill");
    v.soi = all || has("soi");
    v.roche = all || has("roche");
    v.atmospheres = all || has("atmo");
}

// Developer aid for --explode BODY@FRAME: a shuttle-sized explosion on the surface of BODY, under the camera centre (the camera
// is moved onto the contact point, still following the body), so the effect can be checked without waiting for a crash.
void devExplode(App& app, const std::string& bodyName)
{
    const std::string want = lower(bodyName);
    for (const DisplayBody& b : app.state().bodies) {
        if (lower(b.name) != want) continue;
        Vec3 dir = app.camera.centreWorld() - b.r;
        dir.z = 0.0;
        const double n = norm(dir);
        dir = n > 0.0 ? dir / n : Vec3{0.0, 1.0, 0.0};
        ExplosionSpec spec;
        spec.contact = b.r + dir * (b.radius + 1.0);
        spec.refBody = b.id;
        spec.refPos = b.r;
        spec.refVel = b.v;
        spec.refSpin = b.spinRate;
        spec.normal = dir;
        spec.impactVelocity = dir * -3000.0;
        spec.impactorRadius = 2.0;
        spec.sourceId = b.id;
        spec.time = app.state().t;
        app.explosions.spawn(spec);
        app.camera.setCentreWorld(spec.contact);
        std::cout << "Test explosion on " << b.name << "\n";
        return;
    }
    std::cerr << "--explode: no body named '" << bodyName << "'\n";
}

// ---------------------------------------------------------------------------------------------------------------
// UI glue (M6)

// What the bottom bar drives: the RunControl (Start snapshots the scene, Pause captures the run into the editor, Stop restores
// the snapshot, Save writes the current state) and the sim window's settings.
class AppHost : public ui::RunHost {
public:
    explicit AppHost(App& a) : app_(a) {}

    ui::RunState runState() const override
    {
        switch (app_.rc.state()) {
        case RunState::Running: return ui::RunState::Running;
        case RunState::Paused: return ui::RunState::Paused;
        default: return ui::RunState::Stopped;
        }
    }
    void start() override { runStart(app_); }
    void pause() override { runPause(app_); }
    void stop() override { runStop(app_); }
    void saveCurrentAsScene() override { app_.panel.saveCurrent(); }
    bool canStart() const override { return app_.rc.state() != RunState::Running; }
    bool canPause() const override { return app_.rc.state() == RunState::Running; }
    bool canStop() const override { return app_.rc.state() != RunState::Stopped; }

    double warp() const override { return app_.rc.driver().warp().value(); }
    void setWarp(double w) override { app_.rc.driver().warp().set(w); }
    bool canResumeWarp() const override { return app_.rc.driver().warp().canResume(); }
    double resumeWarpValue() const override { return app_.rc.driver().warp().resumeValue(); }
    void resumeWarp() override { app_.rc.driver().warp().resume(); }

    double displayTime() const override { return app_.rc.driver().targetTime(); }
    bool physicsBehind() const override { return app_.rc.running() && app_.rc.driver().physicsBehind(); }

    int threadCount() const override { return app_.threads; }
    void setThreadCount(int n) override { app_.threads = n; applyPrefs(app_); }
    double controllerTick() const override { return app_.controllerTick; }
    void setControllerTick(double s) override
    {
        if (!std::isfinite(s) || s <= 0.0) {   // the simulation ignores such a value: do not show one it does not use
            app_.say("The controller tick must be a positive number of seconds");
            return;
        }
        app_.controllerTick = s;
        applyPrefs(app_);
    }
    bool infiniteFuel() const override { return app_.infiniteFuel; }
    void setInfiniteFuel(bool on) override { app_.infiniteFuel = on; applyPrefs(app_); }
    bool slowDownAnyShuttle() const override { return app_.slowDown.options.anyShuttle; }
    void setSlowDownAnyShuttle(bool on) override { app_.slowDown.options.anyShuttle = on; }

private:
    App& app_;
};

// One label per visible object (plan 9.4 step 10), in ImGui screen coordinates (window units, y down).
void buildLabels(const App& app, const RenderFrame& frame, std::vector<ui::LabelItem>& out)
{
    out.clear();
    const DisplayState& st = app.state();
    const Camera& cam = app.camera;
    const double mpp = cam.metresPerPixel();
    const double toUi = 1.0 / app.pixelScale();   // framebuffer pixels -> window units
    const Follow f = cam.following();
    auto toScreen = [&](const Vec3& world) {
        const Camera::Rel r = cam.toCameraRelative(world);
        return ImVec2(static_cast<float>((0.5 * app.fbW + r.x / mpp) * toUi), static_cast<float>((0.5 * app.fbH - r.y / mpp) * toUi));
    };
    for (const DisplayBody& b : st.bodies) {
        if (b.absorbed) continue;
        ui::LabelItem it;
        it.pos = toScreen(b.r);
        it.radiusPx = static_cast<float>(drawnRadiusPx(b.radius, mpp, frame.bodyMinPixels) * toUi);
        it.text = b.name;
        it.colour = ImVec4(0.45f + 0.55f * b.colour[0], 0.45f + 0.55f * b.colour[1], 0.45f + 0.55f * b.colour[2], 1.0f);
        it.priority = 100.0f + static_cast<float>(std::log10(std::max(b.mu, 1.0)));   // heavier bodies win, bodies beat shuttles
        it.highlighted = (f.kind == TargetKind::Body && f.id == b.id) || app.rc.editor().selected() == b.id;
        out.push_back(std::move(it));
    }
    for (const DisplayShuttle& s : st.shuttles) {
        if (s.dying) continue;
        ui::LabelItem it;
        it.pos = toScreen(s.r);
        const double hullPx = std::max(static_cast<double>(frame.shuttleMinPixels), frame.shuttleLengthMetres / mpp);
        it.radiusPx = static_cast<float>(0.5 * hullPx * toUi);
        it.text = nameOf(app, s.id);
        it.colour = ui::palette().textDim;
        it.priority = 0.0f;
        it.shuttle = true;
        it.highlighted = (f.kind == TargetKind::Shuttle && f.id == s.id) || app.rc.editor().selected() == s.id;
        out.push_back(std::move(it));
    }
}

// Conservation sample for the stats charts: G times E, P, L at the physics time with the merge losses added back, and
// the momentum scale sum(mu |v|).
void sampleConservation(App& app)
{
    const Simulation& sim = app.rc.driver().sim();
    if (sim.time() == app.lastStatSimTime) return;   // paused: nothing new to plot
    app.lastStatSimTime = sim.time();
    const Bodies& b = sim.bodies();
    double scale = 0.0;
    for (std::size_t i = 0; i < b.size(); ++i)
        scale += b.mu[i] * std::sqrt(b.state.vx[i] * b.state.vx[i] + b.state.vy[i] * b.state.vy[i] + b.state.vz[i] * b.state.vz[i]);
    app.bar.stats.recordConserved(sim.time(), sim.conservedWithMerges(), scale);
}

// Event log line for a due event.
void logEvent(App& app, const Event& e)
{
    const std::string when = "[" + formatSimDate(e.t) + "] ";
    switch (e.type) {
    case EventType::Collision:
        app.say(when + "Collision: " + nameOf(app, e.id) + " and " + nameOf(app, e.other) + " at " + formatSpeed(e.relSpeed));
        break;
    case EventType::Merge:
        app.say(when + "Merge: " + nameOf(app, e.id) + " absorbed " + nameOf(app, e.other));
        break;
    case EventType::ShuttleDestroyed:
        app.say(when + nameOf(app, e.id) + " destroyed" + (e.other ? " by " + nameOf(app, e.other) : std::string()));
        break;
    case EventType::AtmosphereEntry:
        app.say(when + nameOf(app, e.id) + " entered the atmosphere of " + nameOf(app, e.other));
        break;
    case EventType::KindChanged:
        app.say(when + nameOf(app, e.id) + " is now a " + kindName(e.newKind));
        break;
    case EventType::FuelEmpty:
        app.say(when + nameOf(app, e.id) + " is out of fuel");
        break;
    }
}

std::string frameLabel(const App& app)
{
    switch (app.camera.frameMode()) {
    case FrameMode::Inertial: return "inertial";
    case FrameMode::Barycentre: return "barycentre";
    case FrameMode::FollowBody: return "rel. " + nameOf(app, app.camera.frameBody());
    }
    return "";
}

// The M4 readout: warp, achieved rate, date, followed object (the ImGui bottom bar replaces it in M6).
std::string buildTitle(const App& app, double achievedRate)
{
    const SimDriver& d = app.rc.driver();
    std::string t = "AWL Cosmos | " + formatSimDate(d.targetTime()) + " | ";
    switch (app.rc.state()) {
    case RunState::Stopped: t += "STOPPED"; break;
    case RunState::Paused: t += "PAUSED"; break;
    case RunState::Running: t += "warp " + formatSig(d.warp().value(), 3) + "x (" + formatTimeRate(achievedRate) + ")"; break;
    }
    const Follow f = app.camera.following();
    t += " | follow: " + (f.active() ? nameOf(app, f.id) : std::string("none"));
    t += " | frame: " + frameLabel(app);
    t += " | " + formatDistance(app.camera.metresPerPixel()) + "/px";
    if (app.rc.running() && d.physicsBehind()) t += " | PHYSICS BEHIND";
    if (glfwGetTime() < app.noticeUntil) t += " | " + app.notice;
    return t;
}

void printHelp()
{
    std::cout << "Keys: wheel = zoom at cursor, drag = pan, click = follow, +/- PgUp/PgDn = zoom, . / , = warp up/down, / = 1x, Space = pause,\n"
                 "      1-9 = follow body n (4 = Domus), F / Tab (+Shift) = cycle follow target, Esc = stop following, Ctrl+Q = quit,\n"
                 "      0 / B / R = inertial / barycentre / relative-to-followed-body frame, H = fit the system\n";
}

// ---------------------------------------------------------------------------------------------------------------
// Scripted editor and run commands (--do COMMAND@FRAME): the same code paths as the panel and the bottom bar, so a headless run
// can build a scene, run it, stop it, save it and load it again. Returns false (and prints why) when a command fails.

bool lookupKind(const std::string& name, BodyKind& out)
{
    const std::string w = lower(name);
    for (BodyKind k : {BodyKind::Star, BodyKind::Planet, BodyKind::GasGiant, BodyKind::DwarfPlanet, BodyKind::Moon,
                       BodyKind::Asteroid, BodyKind::NeutronStar, BodyKind::BlackHole}) {
        std::string n = lower(kindName(k));
        n.erase(std::remove(n.begin(), n.end(), ' '), n.end());
        if (w == n) { out = k; return true; }
    }
    return false;
}

bool lookupField(const std::string& name, app::Field& out)
{
    static const struct { const char* key; app::Field f; } table[] = {
        {"name", app::Field::Name}, {"mass", app::Field::Mass}, {"radius", app::Field::Radius}, {"spin", app::Field::SpinPeriod},
        {"atmorho", app::Field::AtmoRho0}, {"atmoh", app::Field::AtmoScaleHeight}, {"posx", app::Field::PosX},
        {"posy", app::Field::PosY}, {"velx", app::Field::VelX}, {"vely", app::Field::VelY}, {"a", app::Field::OrbitA},
        {"e", app::Field::OrbitE}, {"omega", app::Field::OrbitOmega}, {"m", app::Field::OrbitMeanAnomaly},
        {"dry", app::Field::DryMass}, {"fuel", app::Field::FuelMass}, {"thrust", app::Field::MaxThrust},
        {"exhaust", app::Field::ExhaustVelocity}, {"cda", app::Field::CdA}, {"collision", app::Field::CollisionRadius},
        {"torque", app::Field::MaxTorque}, {"inertia", app::Field::Inertia}};
    const std::string w = lower(name);
    for (const auto& t : table)
        if (w == t.key) { out = t.f; return true; }
    return false;
}

std::uint64_t findObjectByName(const app::EditorModel& ed, const std::string& name)
{
    const std::string w = lower(name);
    for (const SceneBody& b : ed.scene().bodies)
        if (lower(b.name) == w) return b.id;
    for (const SceneShuttle& s : ed.scene().shuttles)
        if (lower(s.name) == w) return s.id;
    return 0;
}

void printReport(App& app)
{
    const DisplayState& st = app.state();
    const app::EditorModel& ed = app.rc.editor();
    std::cout << "REPORT state=" << app::runStateName(app.rc.state()) << " scene='" << ed.scene().name << "' date=" << formatSimDate(st.t)
              << " bodies=" << st.bodies.size() << " shuttles=" << st.shuttles.size() << " editorBodies=" << ed.scene().bodies.size()
              << " editorShuttles=" << ed.scene().shuttles.size() << " selected=" << ed.selected() << " snapshot="
              << (app.rc.snapshot() ? "yes" : "no") << " dirty=" << (app.rc.dirty() ? "yes" : "no") << "\n";
    for (const DisplayBody& b : st.bodies) {
        std::cout << "  body " << b.id << " '" << b.name << "' " << kindName(b.kind) << " mu=" << b.mu << " R=" << b.radius << " r=("
                  << b.r.x << ", " << b.r.y << ") v=(" << b.v.x << ", " << b.v.y << ")\n";
    }
    for (const DisplayShuttle& s : st.shuttles)
        std::cout << "  shuttle " << s.id << " '" << nameOf(app, s.id) << "' r=(" << s.r.x << ", " << s.r.y << ") v=(" << s.v.x << ", " << s.v.y << ")\n";
}

bool runCommand(App& app, const std::string& text)
{
    app::EditorModel& ed = app.rc.editor();
    const std::size_t colon = text.find(':');
    const std::string verb = lower(text.substr(0, colon));
    const std::string arg = colon == std::string::npos ? std::string() : text.substr(colon + 1);
    auto fail = [&](const std::string& why) {
        std::cerr << "COMMAND FAILED [" << text << "]: " << why << "\n";
        return false;
    };
    auto edit = [&](const app::EditResult& r) {
        if (!r) return fail(r.error);
        for (const std::string& w : r.warnings) std::cout << "  warning: " << w << "\n";
        return true;
    };

    if (verb == "new") return app.panel.newScene(arg) || fail(app.panel.sceneMessage());
    if (verb == "load") return app.panel.loadSceneFile(arg) || fail(app.panel.sceneMessage());
    if (verb == "save") return app.panel.saveCurrent(arg) || fail(app.panel.sceneMessage());
    if (verb == "start") { runStart(app); return app.rc.running() || fail("not running"); }
    if (verb == "resumewarp") return app.rc.driver().warp().resume() || fail("nothing to resume");   // the Resume warp button (M7)
    if (verb == "pause") { runPause(app); return app.rc.state() == RunState::Paused || fail("not paused"); }
    if (verb == "stop") { runStop(app); return app.rc.state() == RunState::Stopped || fail("not stopped"); }
    if (verb == "report") { printReport(app); return true; }
    if (verb == "delete") return app.panel.deleteSelected() || fail("nothing deleted");
    if (verb == "warp") { app.rc.driver().warp().set(std::atof(arg.c_str())); return true; }
    if (verb == "add") {
        if (lower(arg) == "shuttle") return app.panel.addShuttle() || fail("cannot add");
        BodyKind k;
        if (!lookupKind(arg, k)) return fail("unknown kind '" + arg + "'");
        return app.panel.addBody(k) || fail("cannot add");
    }
    if (verb == "select") {
        const std::uint64_t id = findObjectByName(ed, arg);
        if (id == 0 || !app.panel.selectObject(id)) return fail("no object named '" + arg + "'");
        return true;
    }
    const std::uint64_t sel = ed.selected();
    if (sel == 0) return fail("no object is selected");
    if (verb == "set") {
        const std::size_t eq = arg.find('=');
        app::Field f;
        if (eq == std::string::npos || !lookupField(arg.substr(0, eq), f)) return fail("expected FIELD=TEXT with a known field");
        return edit(app.panel.applyText(sel, f, arg.substr(eq + 1)));
    }
    if (verb == "kind") {
        BodyKind k;
        if (!lookupKind(arg, k)) return fail("unknown kind '" + arg + "'");
        return edit(ed.setKind(sel, k, true));
    }
    if (verb == "controller") {
        ControllerKind k;
        if (!parseControllerKind(arg, k)) return fail("unknown controller '" + arg + "'");
        return edit(ed.setController(sel, k));
    }
    if (verb == "orbit") {   // orbit:PARENT,DISTANCE
        const std::size_t comma = arg.rfind(',');
        if (comma == std::string::npos) return fail("expected PARENT,DISTANCE");
        const std::uint64_t parent = findObjectByName(ed, arg.substr(0, comma));
        if (parent == 0) return fail("no body named '" + arg.substr(0, comma) + "'");
        const app::ParseResult d = app::parseQuantity(arg.substr(comma + 1), app::Quantity::Length, "m");
        if (!d) return fail(d.error);
        return edit(ed.setOrbitAround(sel, parent, d.value));
    }
    return fail("unknown command '" + verb + "'");
}

struct RunStats {
    long frames = 0;
    double wallStart = 0.0, wallEnd = 0.0;
    double cpuSum = 0.0;          // s of advance + fill + draw calls (without buffer swap and event polling)
    double physicsSum = 0.0, physicsMax = 0.0;
    long bodySteps = 0;
    long behindFrames = 0;
};

int run(const Options& opt)
{
#ifndef NDEBUG
    const bool debugContext = true;
#else
    const bool debugContext = opt.glDebug;
#endif

    // ---- Scene and simulation options (plan 11.1, 4.4) ----
    // The starting scene: a preset id ("empty", "lucerna") or a scene file. The default is the Lucerna preset's file.
    Scene scene;
    std::string err;
    const bool isPresetId = opt.scene.find_first_of("/\\.") == std::string::npos;
    if (!(isPresetId ? loadPreset(opt.scene, defaultSceneDir(), scene, err) : loadSceneFromPath(opt.scene, scene, err))) {
        std::cerr << "Cannot load scene '" << opt.scene << "': " << err << "\n";
        return 1;
    }

    RunOptions runOptions;
    SimulationOptions& so = runOptions.sim;
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    so.threads = std::max(1, std::min(16, opt.threads > 0 ? opt.threads : (hw > 0 ? hw : 1)));
    DriverOptions& dopt = runOptions.driver;
    if (opt.fixedDt > 0.0) dopt.physicsBudget_s = 0.0;   // repeatable runs: the physics may take as long as it needs

    // OpenGL 4.5 core (plan 9). KHR_debug is core since 4.3.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, debugContext ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);   // smooth shuttle hull edges (impostors and lines are anti-aliased analytically)

    GLFWwindow* window = glfwCreateWindow(opt.width > 0 ? opt.width : kDefaultWidth, opt.height > 0 ? opt.height : kDefaultHeight,
                                          "AWL Cosmos", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create a window with an OpenGL 4.5 core context\n";
        return -1;
    }

    App app(runOptions);
    app.window = window;
    app.threads = so.threads;
    app.infiniteFuel = scene.settings.infiniteFuel;
    app.controllerTick = scene.settings.controllerTick_s;
    {
        const std::string sceneName = scene.name;
        if (!app.rc.loadScene(std::move(scene), err)) {   // resolves the scene and builds the first driver
            std::cerr << "Cannot resolve scene '" << opt.scene << "': " << err << "\n";
            glfwDestroyWindow(window);
            return 1;
        }
        std::cout << "Scene " << sceneName << " (" << app.rc.editor().scene().bodies.size() << " bodies, "
                  << app.rc.editor().scene().shuttles.size() << " shuttles) loaded, " << so.threads << " threads\n";
        for (const std::string& w : app.rc.buildWarnings()) std::cerr << "Scene warning: " << w << "\n";
    }
    if (opt.benchShuttles > 0) {   // the benchmark fleet, through the editor, then one rebuild of the (stopped) driver
        std::vector<ControllerKind> mix;
        for (std::size_t from = 0; from < opt.benchControllers.size();) {
            std::size_t to = opt.benchControllers.find(',', from);
            if (to == std::string::npos) to = opt.benchControllers.size();
            ControllerKind k;
            if (parseControllerKind(opt.benchControllers.substr(from, to - from), k)) mix.push_back(k);
            else std::cerr << "--bench-controllers: unknown controller '" << opt.benchControllers.substr(from, to - from) << "'\n";
            from = to + 1;
        }
        const EditResult added = addBenchShuttles(app.rc.editor(), opt.benchShuttles, opt.benchFuel, nullptr, &mix);
        const EditResult applied = added ? app.rc.applyEdits() : added;
        if (!applied) {
            std::cerr << "Cannot add the benchmark shuttles: " << applied.error << "\n";
            glfwDestroyWindow(window);
            return 1;
        }
        std::cout << "Benchmark: " << opt.benchShuttles << " shuttles added (" << opt.benchFuel << " kg fuel each); scene now has "
                  << app.rc.editor().scene().shuttles.size() << " shuttles\n";
    }
    glfwSetWindowUserPointer(window, &app);
    glfwMakeContextCurrent(window);   // before gladLoadGLLoader
    glfwSwapInterval(opt.vsync ? 1 : 0);
    glfwGetFramebufferSize(window, &app.fbW, &app.fbH);
    glfwGetWindowSize(window, &app.winW, &app.winH);
    app.cursorX = 0.5 * app.winW;
    app.cursorY = 0.5 * app.winH;

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }
    GLint samples = 0;
    glGetIntegerv(GL_SAMPLES, &samples);
    std::cout << "OpenGL " << glGetString(GL_VERSION) << " | " << glGetString(GL_RENDERER) << " | MSAA x" << samples << "\n";

    int exitCode = 0;
    {
        std::unique_ptr<render::Renderer> renderer = render::createGlRenderer();
        render::RendererConfig config;
        config.shaderDir = "shaders";
        config.debugOutput = debugContext;
        if (!renderer->init(config)) {
            std::cerr << "Renderer initialisation failed, ABORTING.\n";
            glfwDestroyWindow(window);
            return -1;
        }
        renderer->resize(app.fbW, app.fbH);

        app.rc.driver().warp().set(opt.warp);
        app.seenGeneration = app.rc.generation();
        applyPrefs(app);
        buildSources(app);
        rebuildLookups(app);
        fillNumberKeys(app);
        printHelp();

        // The panel's hooks into the app: camera focus, the log, the view reset after a scene was loaded, the spawn point
        app.panel.hooks.say = [&app](const std::string& text) { app.say(text); };
        app.panel.hooks.focus = [&app](std::uint64_t id, bool shuttle, bool frame) {
            const TargetKind kind = shuttle ? TargetKind::Shuttle : TargetKind::Body;
            app.camera.follow(kind, id);
            if (kind == TargetKind::Body) app.lastFollowedBody = id;
            app.pendingFrame = frame ? App::PendingFrame{kind, id, 0} : App::PendingFrame{};
        };
        app.panel.hooks.sceneReplaced = [&app]() { onSceneReplaced(app); };
        app.panel.hooks.spawnPosition = [&app]() {
            Vec3 c = app.camera.centreWorld();
            c.z = 0.0;
            return c;
        };

        // Start view: the whole system, or the --follow target framed; --mpp overrides the zoom.
        app.camera.setMetresPerPixel(fitSystemMpp(app));
        if (!opt.follow.empty()) followByName(app, opt.follow, true);
        if (opt.mpp > 0.0) app.camera.setMetresPerPixel(opt.mpp);
        app.camera.update(app.state().t, app.src);
        applyOverlayFlag(app.bar.overlays, opt.overlays);
        app.workers.resize(so.threads);
        app.overlays.setThreadPool(&app.workers);
        app.overlays.setShuttlePalette(&app.palette);
        app.overlays.setPredictorOptions(runOptions.sim);   // the predicted path integrates like the live simulation

        glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
        glfwSetScrollCallback(window, scrollCallback);
        glfwSetCursorPosCallback(window, cursorPosCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetKeyCallback(window, keyCallback);

        // ImGui comes AFTER our callbacks: it installs its own and chains to ours (plan 10.4)
        AppHost host(app);
        if (opt.ui) {
            ui::UiConfig uc;
            uc.scaleOverride = opt.uiScale;
            if (!app.ui.init(window, uc)) std::cerr << "UI initialisation failed: running without the UI\n";
            const struct { const char* name; bool* flag; } windows[] = {
                {"view", &app.bar.toolbar.view}, {"sim", &app.bar.toolbar.sim}, {"stats", &app.bar.toolbar.stats},
                {"edit", &app.bar.toolbar.edit}, {"add", &app.bar.toolbar.add}};
            for (const auto& w : windows)
                if (opt.open.find(w.name) != std::string::npos) *w.flag = true;
            if (opt.open.find("nolabels") != std::string::npos) app.bar.showLabels = false;
        }
        std::vector<ui::LabelItem> labelItems;
        int labelsDrawn = 0;
        double lastStatSample = -1.0, lastWall = glfwGetTime();

        RenderFrame frame;
        frame.clearColour[0] = 0.02f; frame.clearColour[1] = 0.022f; frame.clearColour[2] = 0.035f;
        frame.shuttleLengthMetres = 2.0f;   // real 2 m shuttles; the 8 px minimum applies when zoomed out

        RunStats rs;
        double lastTime = glfwGetTime();
        double lastReport = lastTime, lastTitle = -1.0;
        int framesAtReport = 0;
        double titleSim = 0.0, titleReal = 0.0, achievedRate = 0.0;   // rate readout averaged over the title interval
        std::vector<std::uint8_t> pixels;
        PerfSeries perf;             // samples of the frames after the warm-up (--bench-warmup)
        double prevSwapEnd = 0.0;
        std::uint64_t gpuSamplesSeen = 0;

        while (!glfwWindowShouldClose(window) && !app.quit) {
            const double frameStart = glfwGetTime();
            const double realDt = opt.fixedDt > 0.0 ? opt.fixedDt : (opt.benchPaceFps > 0.0 ? 1.0 / opt.benchPaceFps : frameStart - lastTime);
            lastTime = frameStart;
            if (app.dirty) {
                renderer->resize(app.fbW, app.fbH);
                app.dirty = false;
            }

            for (const Options::Click& c : opt.clicks) {   // scripted mouse (--click): move, press, release on consecutive frames
                const int step = static_cast<int>(rs.frames + 1) - c.frame;
                if (step == 0) app.ui.injectMouse(window, c.x, c.y, -1, 0);
                else if (step == 1) app.ui.injectMouse(window, c.x, c.y, GLFW_MOUSE_BUTTON_LEFT, GLFW_PRESS);
                else if (step == 2) app.ui.injectMouse(window, c.x, c.y, GLFW_MOUSE_BUTTON_LEFT, GLFW_RELEASE);
            }
            if (app.ui.initialised()) {   // typed characters and Enter for the focused ImGui field
                ImGuiIO& io = ImGui::GetIO();
                for (const Options::Typed& t : opt.typed) {
                    const int step = static_cast<int>(rs.frames + 1) - t.frame;
                    if (!t.enter && step == 0) io.AddInputCharactersUTF8(t.text.c_str());
                    if (t.enter && step == 0) io.AddKeyEvent(ImGuiKey_Enter, true);
                    if (t.enter && step == 1) io.AddKeyEvent(ImGuiKey_Enter, false);
                }
            }
            for (const Options::Command& c : opt.commands)   // scripted editor / run commands (--do)
                if (c.frame == rs.frames + 1 && !runCommand(app, c.text)) exitCode = 1;
            if (opt.slowDownAt >= 0 && rs.frames + 1 == opt.slowDownAt) app.rc.driver().warp().slowDownOneLevel();
            for (const Options::Scripted& sc : opt.script) {   // scripted input (--press, --scroll) acts before the frame's update
                if (sc.frame != rs.frames + 1) continue;
                if (sc.scroll) scrollCallback(window, 0.0, sc.steps);
                else keyCallback(window, sc.key, 0, GLFW_PRESS, sc.mods);
            }

            // Clock, physics ahead, state at the display time, events (plan 4.4)
            // (RunControl steps the driver only while Running; otherwise it applies the pending edits)
            const RunState stateBefore = app.rc.state();
            app.frameNo = static_cast<long long>(rs.frames) + 1;
            const bool advanced = app.rc.advance(realDt);
            const double tAdvance = glfwGetTime();
            if (stateBefore == RunState::Running && app.rc.state() != RunState::Running)   // the integrator failed: auto-paused
                app.say(app.rc.lastError().empty() ? std::string("Paused") : app.rc.lastError());
            syncGeneration(app);
            static const FrameInfo idleInfo{};
            const FrameInfo fi = advanced ? app.rc.driver().frameInfo() : idleInfo;   // a copy: the UI may replace the driver later in the frame
            if (advanced)
                for (const Event& e : app.rc.driver().dueEvents())
                    if (e.type == EventType::Merge) app.survivorOf[e.other] = e.id;
            rebuildLookups(app);
            if (app.pendingFrame.kind != TargetKind::None) {   // "frame this object" once it exists in the driver's state
                const App::PendingFrame pf = app.pendingFrame;
                const bool exists = pf.kind == TargetKind::Body ? app.bodyIndex.count(pf.id) > 0 : app.shuttleIndex.count(pf.id) > 0;
                if (exists) {
                    const double mpp = framingMpp(app, pf.kind, pf.id);
                    if (mpp > 0.0) app.camera.setMetresPerPixel(mpp);
                    app.pendingFrame = {};
                } else if (++app.pendingFrame.tries > 4) {
                    app.pendingFrame = {};
                }
            }
            if (advanced)
                for (const Event& e : app.rc.driver().dueEvents()) logEvent(app, e);
            // M7: explosions for the events that became due, and the warp step-down (the integrator may move this block).
            if (advanced) {
                for (const Event& e : app.rc.driver().dueEvents()) {
                    app.overlays.onEvent(e);
                    // The followed shuttle crashed: follow the body it hit, so the explosion (on its surface) stays on screen.
                    // Snap to the body rather than keep the view: the camera centre is from the previous frame, which at high
                    // warp is a whole frame of orbital motion away (~50,000 km at 1e5x) and would leave the body off-screen.
                    if (e.type == EventType::ShuttleDestroyed && e.other != 0 && app.camera.following().kind == TargetKind::Shuttle &&
                        app.camera.following().id == e.id && app.bodyIndex.count(e.other)) {
                        app.camera.follow(TargetKind::Body, e.other);
                        app.lastFollowedBody = e.other;
                    }
                }
                app.explosions.handleEvents(app.rc.driver().dueEvents(), app.state(), app.rc.driver().sim());
                std::vector<std::uint64_t> watched;   // the followed shuttle and the selected object (an id that is not a shuttle is ignored)
                if (app.camera.following().kind == TargetKind::Shuttle) watched.push_back(app.camera.following().id);
                if (app.rc.editor().selected() != 0) watched.push_back(app.rc.editor().selected());
                const SlowDownReport sd = app.slowDown.update(app.rc.driver(), watched);
                if (sd.steppedDown)
                    app.say(slowDownMessage(sd, [&app](std::uint64_t id) { return nameOf(app, id); }, formatTimeRate(sd.warpAfter)));
            }
            if (opt.explodeAt > 0 && rs.frames + 1 == opt.explodeAt) devExplode(app, opt.explode);   // --explode (developer aid)
            titleSim += fi.simDelta;
            titleReal += fi.frameDt;
            rs.physicsSum += fi.physicsSeconds;
            rs.physicsMax = std::max(rs.physicsMax, fi.physicsSeconds);
            rs.bodySteps += fi.bodySteps;
            if (fi.physicsBehind) ++rs.behindFrames;

            const CameraUpdate cu = app.camera.update(app.state().t, app.src);
            if (cu.followTransferred) app.say("Following the survivor: " + nameOf(app, app.camera.following().id));
            if (cu.followLost) app.say("Followed object is gone: camera stopped");
            if (cu.frameTransferred) app.say("Reference-frame body merged: frame now follows the survivor");
            if (cu.frameFellBack) app.say("Reference-frame body is gone: frame is inertial");

            if (frameStart - lastTitle >= kTitleInterval_s) {
                if (titleReal > 0.0) achievedRate = titleSim / titleReal;
                titleSim = titleReal = 0.0;
                lastTitle = frameStart;
                glfwSetWindowTitle(window, buildTitle(app, achievedRate).c_str());
            }

            const double tBook = glfwGetTime();
            double tFill = tBook, tOverlays = tBook, tExplosions = tBook, tRenderFill = tBook, tRenderDraw = tBook, tUi = tBook;
            if (app.fbW > 0 && app.fbH > 0) {
                frame.time = opt.fixedDt > 0.0 ? static_cast<double>(rs.frames) * opt.fixedDt : frameStart;   // repeatable flicker with --fixed-dt
                fillFrame(app, frame);
                tFill = glfwGetTime();
                fillOverlays(app, frame);
                tOverlays = glfwGetTime();
                app.explosions.update(app.state(), fi.simDelta, fi.frameDt,
                                      [&app](const Vec3& w, float& x, float& y) {
                                          const Camera::Rel r = app.camera.toCameraRelative(w);
                                          x = r.x;
                                          y = r.y;
                                      },
                                      frame.explosions);
                tExplosions = glfwGetTime();
                renderer->gpuFrameBegin();
                renderer->beginFrame(frame);
                tRenderFill = glfwGetTime();
                renderer->drawGravityWells();
                renderer->drawAtmospheres();
                renderer->drawLines(render::LinePass::Underlay);
                renderer->drawCircles();
                renderer->drawBodies();
                renderer->drawShuttles();
                renderer->drawExplosions();
                renderer->drawLines(render::LinePass::Overlay);
                renderer->endFrame();
                tRenderDraw = glfwGetTime();

                // UI last (plan 9.4 steps 10-11): labels in the foreground draw list, then the ImGui windows
                if (app.ui.initialised()) {
                    if (opt.fixedDt > 0.0 || frameStart - lastStatSample >= kStatsInterval_s) {
                        sampleConservation(app);
                        lastStatSample = frameStart;
                    }
                    app.bar.stats.recordFrame((frameStart - lastWall) * 1000.0, fi.physicsSeconds * 1000.0);
                    lastWall = frameStart;
                    app.ui.newFrame();
                    const ImVec2 display = ImGui::GetIO().DisplaySize;
                    app.bar.draw(host, display, app.ui.fontMedium());
                    app.panel.draw(app.bar.toolbar, display, app.bar.height());
                    app.labelOptions.show = app.bar.showLabels;
                    app.labelOptions.keepOut = app.bar.occupied();
                    app.labelOptions.keepOut.insert(app.labelOptions.keepOut.end(), app.panel.occupied().begin(), app.panel.occupied().end());
                    buildLabels(app, frame, labelItems);
                    labelsDrawn = ui::drawLabels(labelItems, app.labelOptions, app.ui.dpiScale());
                    app.ui.render();
                }
                tUi = glfwGetTime();
                renderer->gpuFrameEnd();

                ++rs.frames;
                if (rs.frames == 1) rs.wallStart = glfwGetTime();   // the first frame includes shader and buffer warm-up
                const bool lastFrame = opt.frames >= 0 && rs.frames >= opt.frames;
                auto shoot = [&](const std::string& path) {
                    int w = 0, h = 0;
                    if (renderer->readBackBuffer(pixels, w, h) && writeBmp24(path, w, h, pixels.data()))
                        std::cout << "Wrote " << path << " (" << w << "x" << h << ")\n";
                    else { std::cerr << "Screenshot failed\n"; exitCode = 1; }
                };
                for (const Options::Shot& sh : opt.shots)
                    if (sh.frame == rs.frames) shoot(sh.path);
                if (lastFrame && !opt.screenshot.empty()) shoot(opt.screenshot);
                if (opt.frames >= 2 && rs.frames == opt.frames - 1 && !opt.screenshotPrev.empty()) shoot(opt.screenshotPrev);
                if (rs.frames > 1) rs.cpuSum += glfwGetTime() - frameStart;
                const double cpuEnd = glfwGetTime();
                glfwSwapBuffers(window);
                rs.wallEnd = glfwGetTime();
                // Statistics of the measured frames (after the warm-up); the frame time is swap return to swap return.
                if (rs.frames > 1 && rs.frames > opt.benchWarmup && prevSwapEnd > 0.0) {
                    auto msOf = [](double a, double b) { return (b - a) * 1000.0; };
                    perf.frame.push_back(msOf(prevSwapEnd, rs.wallEnd));
                    perf.cpuWork.push_back(msOf(frameStart, cpuEnd));
                    perf.physics.push_back(fi.physicsSeconds * 1000.0);
                    perf.bodiesIas15.push_back(fi.bodiesSeconds * 1000.0);
                    perf.shuttlePass.push_back(fi.shuttleSeconds * 1000.0);
                    perf.driverState.push_back(fi.stateSeconds * 1000.0);
                    perf.phase[static_cast<std::size_t>(Phase::Advance)].push_back(msOf(frameStart, tAdvance));
                    perf.phase[static_cast<std::size_t>(Phase::Bookkeeping)].push_back(msOf(tAdvance, tBook));
                    perf.phase[static_cast<std::size_t>(Phase::FillFrame)].push_back(msOf(tBook, tFill));
                    perf.phase[static_cast<std::size_t>(Phase::FillOverlays)].push_back(msOf(tFill, tOverlays));
                    perf.phase[static_cast<std::size_t>(Phase::Explosions)].push_back(msOf(tOverlays, tExplosions));
                    perf.phase[static_cast<std::size_t>(Phase::RenderFill)].push_back(msOf(tExplosions, tRenderFill));
                    perf.phase[static_cast<std::size_t>(Phase::RenderDraw)].push_back(msOf(tRenderFill, tRenderDraw));
                    perf.phase[static_cast<std::size_t>(Phase::Ui)].push_back(msOf(tRenderDraw, tUi));
                    perf.phase[static_cast<std::size_t>(Phase::Swap)].push_back(msOf(cpuEnd, rs.wallEnd));
                    perf.phase[static_cast<std::size_t>(Phase::Poll)].push_back(0.0);   // set after glfwPollEvents below
                    const OverlayStats& ov = app.overlays.stats();
                    perf.ovTrailRecord.push_back(ov.trailRecordMs);
                    perf.ovTrailDraw.push_back(ov.trailDrawMs);
                    perf.ovOrbits.push_back(ov.orbitsMs);
                    perf.ovPredict.push_back(ov.predictMs);
                    perf.ovArrows.push_back(ov.arrowsMs);
                    perf.ovWellsEtc.push_back(app.wellsEtcMs);
                    const render::RenderTotals& tt = renderer->totals();
                    perf.gpu.push_back(tt.frameGpuSamples > gpuSamplesSeen ? tt.frameGpuMsLast : (perf.gpu.empty() ? 0.0 : perf.gpu.back()));
                    gpuSamplesSeen = tt.frameGpuSamples;
                }
                prevSwapEnd = rs.wallEnd;
                if (lastFrame) break;
            }

            const double now = glfwGetTime();
            if (now - lastReport >= 1.0) {
                std::cout << "fps " << (rs.frames - framesAtReport) / (now - lastReport) << "\n";
                lastReport = now;
                framesAtReport = static_cast<int>(rs.frames);
            }
            if (app.fbW > 0 && app.fbH > 0) {
                const double tPollStart = glfwGetTime();
                glfwPollEvents();
                auto& pp = perf.phase[static_cast<std::size_t>(Phase::Poll)];
                if (!pp.empty() && pp.size() == perf.frame.size()) pp.back() = (glfwGetTime() - tPollStart) * 1000.0;
            } else glfwWaitEventsTimeout(0.1);   // minimised: nothing is drawn or swapped, so block instead of spinning a core
        }

        if (rs.frames > 1) {
            const double n = static_cast<double>(rs.frames - 1);
            const double avg = (rs.wallEnd - rs.wallStart) / n;
            std::cout << "Frames " << rs.frames << ": avg frame " << avg * 1000.0 << " ms (" << 1.0 / avg << " fps), avg CPU work "
                      << rs.cpuSum / n * 1000.0 << " ms, physics avg " << rs.physicsSum / static_cast<double>(rs.frames) * 1000.0
                      << " ms max " << rs.physicsMax * 1000.0 << " ms, body steps " << rs.bodySteps << ", physics-behind frames "
                      << rs.behindFrames << ", sim time " << formatSimDate(app.rc.driver().targetTime()) << "\n";
        }
        if (!perf.frame.empty()) {
            std::string label = opt.benchLabel.empty() ? std::string("run") : opt.benchLabel;
            label += " [" + std::to_string(app.fbW) + "x" + std::to_string(app.fbH) + ", warp " + formatSig(opt.warp, 3) + ", " +
                     std::to_string(app.rc.driver().sim().shuttles().aliveCount()) + " shuttles alive, " + std::to_string(so.threads) +
                     " threads, vsync " + (opt.vsync ? "on" : "off") + ", warm-up " + std::to_string(opt.benchWarmup) + "]";
            std::cout << perfReport(perf, label);
            const SimulationStats& ss = app.rc.driver().sim().stats();
            std::cout << "  simulation totals: body steps " << ss.bodySteps << ", shuttle steps " << ss.shuttleSteps << " (rejected "
                      << ss.shuttleRejects << "), force calls " << ss.shuttleForceCalls << ", controller ticks " << ss.controllerTicks
                      << ", destroyed " << ss.shuttlesDestroyed << '\n';
            if (!opt.perfCsv.empty()) {
                if (FILE* f = std::fopen(opt.perfCsv.c_str(), "wb")) {
                    const std::string csv = perfCsv(perf);
                    std::fwrite(csv.data(), 1, csv.size(), f);
                    std::fclose(f);
                } else {
                    std::cerr << "Cannot write " << opt.perfCsv << '\n';
                }
            }
        }
        std::cout << "Final readout: " << buildTitle(app, achievedRate) << "\n";
        const render::RenderStats& st = renderer->frameStats();
        const render::RenderTotals& tot = renderer->totals();
        std::cout << "Last frame: draw calls " << st.drawCalls << " (bodies " << st.bodyInstances << ", shuttles " << st.shuttleInstances
                  << ", line segments " << st.lineSegments << ", explosion draws " << st.explosionDrawCalls << " over " << st.explosionFragments << " fragments), lights " << st.lightCount << ", streamed " << st.streamedBytes << " bytes\n";
        if (tot.wellsGpuSamples > 0)
            std::cout << "Gravity-well pass GPU time: avg " << tot.wellsGpuMsSum / static_cast<double>(tot.wellsGpuSamples) << " ms over "
                      << tot.wellsGpuSamples << " samples (last " << tot.wellsGpuMsLast << " ms)\n";
        std::cout << "UI: " << (app.ui.initialised() ? "on" : "off") << ", labels drawn in the last frame " << labelsDrawn
                  << ", UI scale " << app.ui.dpiScale() << (app.ui.fontsLoaded() ? "" : " (fonts missing)") << "\n";
        std::cout << "Totals: frames " << tot.frames << ", GL errors " << tot.glErrors << ", GL warnings " << tot.glWarnings
                  << ", fence stalls " << tot.fenceStalls << ", buffer growths " << tot.arenaGrowths << "\n";
        if (tot.glErrors > 0) exitCode = 1;
        app.ui.shutdown();   // while the context is current and the window alive (ImGui restores the GLFW callbacks)
        renderer->shutdown();
    }   // renderer destroyed while the context is still current

    glfwDestroyWindow(window);
    return exitCode;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 2;
    if (!glfwInit()) return -1;
    const int rc = run(opt);
    glfwTerminate();
    return rc;
}
