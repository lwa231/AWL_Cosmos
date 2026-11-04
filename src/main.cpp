// AWL Cosmos application (plan M4, M6): loads a scene, runs it through SimDriver (clock + physics ahead + interpolation),
// moves the camera and hands a camera-relative RenderFrame to the renderer. The ImGui UI (bottom bar, labels, stats) is
// drawn after the scene passes; the GLFW callbacks are installed BEFORE ImGui's so ImGui chains to them, and camera
// and keyboard input is gated on ImGui's WantCapture* flags (plan 10.4).
#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "app/bmp_writer.h"
#include "app/camera.h"
#include "app/format.h"
#include "app/palette.h"
#include "app/render_frame.h"
#include "app/sim_driver.h"
#include "core/kinds.h"
#include "core/scene_json.h"
#include "render/renderer.h"
#include "ui/bottom_bar.h"
#include "ui/labels.h"
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
    std::string scene = "scenes/lucerna.json";   // --scene path (relative to the working folder, like shaders/)
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
    std::string screenshotPrev;       // --screenshot-prev path.bmp: also write the frame before the last (needs --frames >= 2)
    // Scripted input for verification without a human: --press KEY@FRAME (a key as in the help text: 0-9, A-Z, '.', ',',
    // '/', '-', '=', Space, Tab, Esc; a leading '+' adds Shift) and --scroll STEPS@FRAME (wheel notches at the cursor,
    // which starts at the window centre).
    struct Scripted { int frame = 0; bool scroll = false; int key = 0; int mods = 0; double steps = 0.0; };
    std::vector<Scripted> script;
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
        else if (a == "--no-ui") o.ui = false;
        else if ((a == "--press" || a == "--scroll") && i + 1 < argc) {
            Options::Scripted sc;
            if (!parseScripted(argv[++i], a == "--scroll", sc)) { std::cerr << "Bad value for " << a << ": " << argv[i] << "\n"; return false; }
            o.script.push_back(sc);
        }
        else if (a == "--no-vsync") o.vsync = false;
        else if (a == "--gl-debug") o.glDebug = true;
        else {
            std::cerr << "Unknown or incomplete option: " << a << "\n"
                      << "Usage: AWL_Cosmos [--scene file.json] [--warp X] [--follow NAME] [--mpp X] [--size W H] [--frames N]\n"
                      << "                  [--screenshot file.bmp] [--screenshot-prev file.bmp] [--fixed-dt S] [--threads N]\n"
                      << "                  [--no-vsync] [--gl-debug] [--no-ui] [--ui-scale X] [--open view,sim,stats,log,edit,add]\n"
                      << "                  [--click X,Y@FRAME]... [--slow-down-at FRAME]\n"
                      << "                  [--press KEY@FRAME]... [--scroll STEPS@FRAME]...\n";
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
    GLFWwindow* window = nullptr;
    std::unique_ptr<SimDriver> driver;
    Camera camera;
    CameraSources src;
    ShuttlePalette palette{2024};
    ui::UiContext ui;                // ImGui context and backends (initialised after the callbacks below are installed)
    ui::BottomBar bar;               // bottom bar, its windows, the event log and the stats history
    ui::LabelOptions labelOptions;
    double lastStatSimTime = -1.0e300;   // physics time of the last conservation sample

    int winW = 0, winH = 0;          // window size in screen coordinates (cursor units)
    int fbW = 0, fbH = 0;            // framebuffer size in pixels
    bool dirty = true;               // framebuffer size changed

    // Lookups rebuilt from driver->state() every frame (ids are unique across bodies and shuttles).
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

    const DisplayState& state() const { return driver->state(); }
    void say(const std::string& s)
    {
        notice = s;
        noticeUntil = glfwGetTime() + kNoticeSeconds;
        bar.log.push(s);
        std::cout << s << "\n";
    }
};

std::string nameOf(const App& app, std::uint64_t id)
{
    auto it = app.names.find(id);
    return it != app.names.end() ? it->second : "#" + std::to_string(id);
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
    const Shuttles& sh = app.driver->sim().shuttles();
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
        return a->driver->sim().bodyPositionAt(id, t, out);
    };
    s.bodyVelocity = [a](std::uint64_t id, double t, Vec3& out) {
        if (t == a->state().t) {
            auto it = a->bodyIndex.find(id);
            if (it == a->bodyIndex.end()) return false;
            out = a->state().bodies[it->second].v;
            return true;
        }
        return a->driver->sim().bodyVelocityAt(id, t, out);
    };
    s.shuttlePosition = [a](std::uint64_t id, double t, Vec3& out) {
        auto it = a->shuttleIndex.find(id);
        if (it == a->shuttleIndex.end()) return false;
        const DisplayShuttle& sh = a->state().shuttles[it->second];
        if (t == a->state().t) { out = sh.r; return true; }
        out = a->driver->sim().shuttlePositionAt(sh.index, t);
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
    if (pr.hit) followTarget(app, pr.kind, pr.id, false);
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
    SimDriver& d = *app->driver;
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
    case GLFW_KEY_SPACE: d.setPaused(!d.paused()); break;
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

// ---------------------------------------------------------------------------------------------------------------
// UI glue (M6)

// What the bottom bar drives. Until the editor's RunControl is bound here, Start / Pause map to the SimDriver clock and
// Stop / Save as scene are disabled (there is no scene snapshot to restore or editor model to save from yet); the warp and
// the simulation settings act on the SimDriver directly.
class AppHost : public ui::RunHost {
public:
    explicit AppHost(App& a) : app_(a) {}

    ui::RunState runState() const override { return app_.driver->paused() ? ui::RunState::Paused : ui::RunState::Running; }
    void start() override { app_.driver->setPaused(false); }
    void pause() override { app_.driver->setPaused(true); }
    void stop() override { app_.driver->setPaused(true); }
    void saveCurrentAsScene() override {}
    bool canStop() const override { return false; }
    bool canSaveCurrent() const override { return false; }

    double warp() const override { return app_.driver->warp().value(); }
    void setWarp(double w) override { app_.driver->warp().set(w); }
    bool canResumeWarp() const override { return app_.driver->warp().canResume(); }
    double resumeWarpValue() const override { return app_.driver->warp().resumeValue(); }
    void resumeWarp() override { app_.driver->warp().resume(); }

    double displayTime() const override { return app_.driver->targetTime(); }
    bool physicsBehind() const override { return app_.driver->physicsBehind(); }

    int threadCount() const override { return app_.driver->sim().threadCount(); }
    void setThreadCount(int n) override { app_.driver->sim().setThreadCount(n); }
    double controllerTick() const override { return app_.driver->sim().controllerTick(); }
    void setControllerTick(double s) override { app_.driver->sim().setControllerTick(s); }
    bool infiniteFuel() const override { return app_.driver->sim().infiniteFuel(); }
    void setInfiniteFuel(bool on) override { app_.driver->sim().setInfiniteFuel(on); }

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
        it.highlighted = f.kind == TargetKind::Body && f.id == b.id;
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
        it.highlighted = f.kind == TargetKind::Shuttle && f.id == s.id;
        out.push_back(std::move(it));
    }
}

// Conservation sample for the stats charts: G times E, P, L at the physics time with the merge losses added back, and
// the momentum scale sum(mu |v|).
void sampleConservation(App& app)
{
    const Simulation& sim = app.driver->sim();
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
    const SimDriver& d = *app.driver;
    std::string t = "AWL Cosmos | " + formatSimDate(d.targetTime()) + " | ";
    t += d.paused() ? std::string("PAUSED") : "warp " + formatSig(d.warp().value(), 3) + "x (" + formatTimeRate(achievedRate) + ")";
    const Follow f = app.camera.following();
    t += " | follow: " + (f.active() ? nameOf(app, f.id) : std::string("none"));
    t += " | frame: " + frameLabel(app);
    t += " | " + formatDistance(app.camera.metresPerPixel()) + "/px";
    if (d.physicsBehind()) t += " | PHYSICS BEHIND";
    if (glfwGetTime() < app.noticeUntil) t += " | " + app.notice;
    return t;
}

void printHelp()
{
    std::cout << "Keys: wheel = zoom at cursor, drag = pan, click = follow, +/- PgUp/PgDn = zoom, . / , = warp up/down, / = 1x, Space = pause,\n"
                 "      1-9 = follow body n (4 = Domus), F / Tab (+Shift) = cycle follow target, Esc = stop following, Ctrl+Q = quit,\n"
                 "      0 / B / R = inertial / barycentre / relative-to-followed-body frame, H = fit the system\n";
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

    // ---- Scene and simulation (plan 11.1, 4.4) ----
    Scene scene;
    std::string err;
    if (!loadSceneFile(opt.scene, scene, err)) {
        std::cerr << "Cannot load scene '" << opt.scene << "': " << err << "\n";
        return 1;
    }
    ResolvedScene resolved;
    if (!resolveScene(scene, resolved, err)) {
        std::cerr << "Cannot resolve scene '" << opt.scene << "': " << err << "\n";
        return 1;
    }
    for (const std::string& w : resolved.warnings) std::cerr << "Scene warning: " << w << "\n";

    SimulationOptions so;
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    so.threads = std::max(1, std::min(16, opt.threads > 0 ? opt.threads : (hw > 0 ? hw : 1)));
    so.startTime = scene.epoch_s;
    DriverOptions dopt;
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

    App app;
    app.window = window;
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

        app.driver = std::make_unique<SimDriver>(resolved, scene.settings, so, dopt);
        app.driver->warp().set(opt.warp);
        for (const std::string& w : app.driver->sim().warnings()) std::cerr << "Simulation warning: " << w << "\n";
        buildSources(app);
        rebuildLookups(app);
        {
            std::vector<std::uint64_t> ids;
            for (const DisplayBody& b : app.state().bodies) ids.push_back(b.id);
            std::sort(ids.begin(), ids.end());
            for (std::size_t k = 0; k < app.numberKeys.size() && k < ids.size(); ++k) app.numberKeys[k] = ids[k];
        }
        std::cout << "Scene " << scene.name << ": " << app.state().bodies.size() << " bodies, " << app.state().shuttles.size()
                  << " shuttles, " << so.threads << " threads\n";
        printHelp();

        // Start view: the whole system, or the --follow target framed; --mpp overrides the zoom.
        app.camera.setMetresPerPixel(fitSystemMpp(app));
        if (!opt.follow.empty()) followByName(app, opt.follow, true);
        if (opt.mpp > 0.0) app.camera.setMetresPerPixel(opt.mpp);
        app.camera.update(app.state().t, app.src);

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

        while (!glfwWindowShouldClose(window) && !app.quit) {
            const double frameStart = glfwGetTime();
            const double realDt = opt.fixedDt > 0.0 ? opt.fixedDt : frameStart - lastTime;
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
            if (opt.slowDownAt >= 0 && rs.frames + 1 == opt.slowDownAt) app.driver->warp().slowDownOneLevel();
            for (const Options::Scripted& sc : opt.script) {   // scripted input (--press, --scroll) acts before the frame's update
                if (sc.frame != rs.frames + 1) continue;
                if (sc.scroll) scrollCallback(window, 0.0, sc.steps);
                else keyCallback(window, sc.key, 0, GLFW_PRESS, sc.mods);
            }

            // Clock, physics ahead, state at the display time, events (plan 4.4)
            const FrameInfo& fi = app.driver->advance(realDt);
            for (const Event& e : app.driver->dueEvents()) {
                if (e.type == EventType::Merge) app.survivorOf[e.other] = e.id;
            }
            rebuildLookups(app);
            for (const Event& e : app.driver->dueEvents()) logEvent(app, e);
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

            if (app.fbW > 0 && app.fbH > 0) {
                frame.time = opt.fixedDt > 0.0 ? static_cast<double>(rs.frames) * opt.fixedDt : frameStart;   // repeatable flicker with --fixed-dt
                fillFrame(app, frame);
                renderer->beginFrame(frame);
                renderer->drawLines(render::LinePass::Underlay);
                renderer->drawBodies();
                renderer->drawShuttles();
                renderer->drawLines(render::LinePass::Overlay);
                renderer->endFrame();

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
                    app.labelOptions.show = app.bar.showLabels;
                    app.labelOptions.keepOut = app.bar.occupied();
                    buildLabels(app, frame, labelItems);
                    labelsDrawn = ui::drawLabels(labelItems, app.labelOptions, app.ui.dpiScale());
                    app.ui.render();
                }

                ++rs.frames;
                if (rs.frames == 1) rs.wallStart = glfwGetTime();   // the first frame includes shader and buffer warm-up
                const bool lastFrame = opt.frames >= 0 && rs.frames >= opt.frames;
                auto shoot = [&](const std::string& path) {
                    int w = 0, h = 0;
                    if (renderer->readBackBuffer(pixels, w, h) && writeBmp24(path, w, h, pixels.data()))
                        std::cout << "Wrote " << path << " (" << w << "x" << h << ")\n";
                    else { std::cerr << "Screenshot failed\n"; exitCode = 1; }
                };
                if (lastFrame && !opt.screenshot.empty()) shoot(opt.screenshot);
                if (opt.frames >= 2 && rs.frames == opt.frames - 1 && !opt.screenshotPrev.empty()) shoot(opt.screenshotPrev);
                if (rs.frames > 1) rs.cpuSum += glfwGetTime() - frameStart;
                glfwSwapBuffers(window);
                rs.wallEnd = glfwGetTime();
                if (lastFrame) break;
            }

            const double now = glfwGetTime();
            if (now - lastReport >= 1.0) {
                std::cout << "fps " << (rs.frames - framesAtReport) / (now - lastReport) << "\n";
                lastReport = now;
                framesAtReport = static_cast<int>(rs.frames);
            }
            if (app.fbW > 0 && app.fbH > 0) glfwPollEvents();
            else glfwWaitEventsTimeout(0.1);   // minimised: nothing is drawn or swapped, so block instead of spinning a core
        }

        if (rs.frames > 1) {
            const double n = static_cast<double>(rs.frames - 1);
            const double avg = (rs.wallEnd - rs.wallStart) / n;
            std::cout << "Frames " << rs.frames << ": avg frame " << avg * 1000.0 << " ms (" << 1.0 / avg << " fps), avg CPU work "
                      << rs.cpuSum / n * 1000.0 << " ms, physics avg " << rs.physicsSum / static_cast<double>(rs.frames) * 1000.0
                      << " ms max " << rs.physicsMax * 1000.0 << " ms, body steps " << rs.bodySteps << ", physics-behind frames "
                      << rs.behindFrames << ", sim time " << formatSimDate(app.driver->targetTime()) << "\n";
        }
        std::cout << "Final readout: " << buildTitle(app, achievedRate) << "\n";
        const render::RenderStats& st = renderer->frameStats();
        const render::RenderTotals& tot = renderer->totals();
        std::cout << "Last frame: draw calls " << st.drawCalls << " (bodies " << st.bodyInstances << ", shuttles " << st.shuttleInstances
                  << ", line segments " << st.lineSegments << "), lights " << st.lightCount << ", streamed " << st.streamedBytes << " bytes\n";
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
