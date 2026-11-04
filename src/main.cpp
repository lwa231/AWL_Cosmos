#include <glad/glad.h>   // must come BEFORE glfw3.h
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "SimulationLogic.h"
#include "app/bmp_writer.h"
#include "app/palette.h"
#include "app/render_frame.h"
#include "render/renderer.h"

// Window
static const int SCREEN_WIDTH = 800;
static const int SCREEN_HEIGHT = 600;

// Until M4 brings the camera, the old toy-unit scene is drawn at a fixed scale: 1 toy unit = 1 "metre", 0.1 per pixel
// (the same 60 units over 600 pixels the original orthographic view showed).
static const float DEMO_METRES_PER_PIXEL = 0.1f;
static const int DEMO_SHUTTLES = 200;
static const double DEMO_CORE_MASS = 120.0;   // matches M_CORE below

struct Options {
    int frames = -1;                 // --frames N: exit after N frames
    std::string screenshot;          // --screenshot path.bmp: write the last frame's back buffer
    bool vsync = true;               // --no-vsync turns it off
    float mpp = DEMO_METRES_PER_PIXEL;   // --mpp X: metres per pixel of the demo view (zoom in to inspect shuttles)
    int width = SCREEN_WIDTH, height = SCREEN_HEIGHT;   // --size W H: initial window size
    int shuttles = DEMO_SHUTTLES;    // --shuttles N: demo shuttle count (stress test of the streaming buffer)
    bool glDebug = false;            // --gl-debug: request a debug context and report GL errors (always on in Debug builds)
};

struct WindowState {
    int width = SCREEN_WIDTH;
    int height = SCREEN_HEIGHT;
    bool dirty = true;
};

static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    auto* state = static_cast<WindowState*>(glfwGetWindowUserPointer(window));
    state->width = width;
    state->height = height;
    state->dirty = true;
}

static bool parseArgs(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) o.frames = std::atoi(argv[++i]);
        else if (a == "--screenshot" && i + 1 < argc) o.screenshot = argv[++i];
        else if (a == "--mpp" && i + 1 < argc) o.mpp = static_cast<float>(std::atof(argv[++i]));
        else if (a == "--size" && i + 2 < argc) { o.width = std::atoi(argv[++i]); o.height = std::atoi(argv[++i]); }
        else if (a == "--shuttles" && i + 1 < argc) o.shuttles = std::atoi(argv[++i]);
        else if (a == "--no-vsync") o.vsync = false;
        else if (a == "--gl-debug") o.glDebug = true;
        else {
            std::cerr << "Unknown or incomplete option: " << a << "\n"
                      << "Usage: AWL_Cosmos [--frames N] [--screenshot file.bmp] [--no-vsync] [--gl-debug] [--mpp X] [--size W H] [--shuttles N]\n";
            return false;
        }
    }
    return true;
}

// Fills the frame from the old toy-unit simulation. M4 replaces this with the real simulation + camera.
static void addOldSimBodies(const SimLogic& sim, cosmos::app::RenderFrame& frame)
{
    const auto& bodies = sim.getBodies();
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const auto& b = bodies[i];
        const KindProps props = kindProps(b.kind);
        cosmos::app::RenderBody rb;
        // Camera-relative: subtract the camera (the origin here) in double, then cast to float (plan 9.3)
        rb.x = static_cast<float>(b.x - 0.0);
        rb.y = static_cast<float>(b.y - 0.0);
        rb.radius = static_cast<float>(b.radius);
        rb.colour = {b.color[0], b.color[1], b.color[2]};
        rb.ambient = props.ambient;
        rb.receivesLight = props.receivesLight;
        rb.emissive = !props.receivesLight;
        if (i == 0) {                       // the pseudo-galaxy's black core
            rb.isBlackHole = true;
            rb.colour = {1.0f, 0.78f, 0.45f};   // ring colour
        }
        frame.bodies.push_back(rb);
    }
}

// Lit planets and a star on analytic circular orbits so the impostor shading and the light buffer are visible.
static void addDemoBodies(double t, cosmos::app::RenderFrame& frame)
{
    struct Orbiter { double r, phase, radius; float cr, cg, cb; bool star; };
    static const Orbiter kOrbiters[] = {
        {23.0, 0.9, 2.4, 1.00f, 0.95f, 0.75f, true},    // star + light
        {13.0, 2.6, 1.8, 0.30f, 0.55f, 0.95f, false},   // blue planet
        {19.0, 4.4, 1.3, 0.85f, 0.45f, 0.30f, false},   // red planet
        { 9.0, 5.5, 0.9, 0.60f, 0.80f, 0.55f, false},   // green planet
    };
    for (const Orbiter& o : kOrbiters) {
        // Orbit circle in the underlay line batch: a polyline whose alpha fades once around (exercises the line path)
        {
            const int N = 160;
            std::vector<cosmos::app::LineVertex> ring(N + 1);
            for (int k = 0; k <= N; ++k) {
                const double a = 6.283185307179586 * k / N;
                cosmos::app::LineVertex v;
                v.x = static_cast<float>(o.r * std::cos(a));
                v.y = static_cast<float>(o.r * std::sin(a));
                v.r = 0.45f; v.g = 0.55f; v.b = 0.8f;
                v.a = 0.12f + 0.5f * static_cast<float>(k) / N;
                ring[k] = v;
            }
            frame.underlay.addStrip(ring, 1.5f);
        }
        const double omega = std::sqrt(DEMO_CORE_MASS / (o.r * o.r * o.r));
        const double a = o.phase + omega * t;
        cosmos::app::RenderBody b;
        b.x = static_cast<float>(o.r * std::cos(a));
        b.y = static_cast<float>(o.r * std::sin(a));
        b.radius = static_cast<float>(o.radius);
        b.colour = {o.cr, o.cg, o.cb};
        b.emissive = o.star;
        b.receivesLight = !o.star;
        b.ambient = 0.05f;
        frame.bodies.push_back(b);
        if (o.star) {
            cosmos::app::RenderLight l;
            l.x = b.x; l.y = b.y;
            l.colour = {0.5f + 0.5f * o.cr, 0.5f + 0.5f * o.cg, 0.5f + 0.5f * o.cb};   // tinted toward white, as before
            l.intensity = 1.3f;
            frame.lights.push_back(l);
        }
    }
}

// 200 shuttles on circular orbits, headings spinning or tangent, throttle pulsing. Colours come from the palette (7.4).
static void addDemoShuttles(double t, int count, const cosmos::app::ShuttlePalette& palette, cosmos::app::RenderFrame& frame)
{
    for (int i = 1; i <= count; ++i) {
        const double r = 3.0 + 23.0 * std::fmod(i * 0.7548776662, 1.0);              // radius and start angle from two
        const double omega = std::sqrt(DEMO_CORE_MASS / (r * r * r));              // unrelated low-discrepancy sequences,
        const double a = 6.283185307 * std::fmod(i * 0.5698402910, 1.0) + omega * t;   // so the swarm fills the disc
        cosmos::app::RenderShuttle s;
        s.x = static_cast<float>(r * std::cos(a));
        s.y = static_cast<float>(r * std::sin(a));
        if (i % 2 == 0) s.heading = static_cast<float>(a + 1.5707963 + 0.3 * std::sin(0.9 * t + i));   // roughly prograde
        else            s.heading = static_cast<float>(i * 0.7 + (0.6 + 0.02 * (i % 17)) * t);          // tumbling
        const auto cols = palette.coloursFor(static_cast<std::uint64_t>(i));
        for (int k = 0; k < 3; ++k) s.sector[k] = {cols[k].r, cols[k].g, cols[k].b};
        s.throttle = static_cast<float>(std::min(1.0, std::max(0.0, 1.4 * std::sin(0.7 * t + i * 0.37))));
        s.flickerPhase = static_cast<float>(i * 1.7);
        frame.shuttles.push_back(s);
    }
}

// Two axes through the origin in the overlay batch, to exercise the second line pass.
static void addDemoOverlay(cosmos::app::RenderFrame& frame)
{
    cosmos::app::LineVertex a, b;
    a.x = -30.0f; a.y = 0.0f; b.x = 30.0f; b.y = 0.0f;
    a.r = b.r = 0.9f; a.g = b.g = 0.3f; a.b = b.b = 0.3f;
    a.a = 0.0f; b.a = 0.8f;
    frame.overlay.addSegment(a, b, 2.0f);
    a.x = 0.0f; a.y = -30.0f; b.x = 0.0f; b.y = 30.0f;
    a.r = b.r = 0.3f; a.g = b.g = 0.9f; a.b = b.b = 0.4f;
    frame.overlay.addSegment(a, b, 2.0f);
}

static int run(const Options& opt)
{
#ifndef NDEBUG
    const bool debugContext = true;
#else
    const bool debugContext = opt.glDebug;
#endif
    // OpenGL 4.5 core (plan 9). KHR_debug is core since 4.3.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, debugContext ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);   // smooth shuttle hull edges (impostors and lines are anti-aliased analytically)

    GLFWwindow* window = glfwCreateWindow(opt.width > 0 ? opt.width : SCREEN_WIDTH, opt.height > 0 ? opt.height : SCREEN_HEIGHT, "AWL Cosmos", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create a window with an OpenGL 4.5 core context\n";
        return -1;
    }

    WindowState windowState;
    glfwSetWindowUserPointer(window, &windowState);
    glfwMakeContextCurrent(window);   // before gladLoadGLLoader
    glfwSwapInterval(opt.vsync ? 1 : 0);
    glfwGetFramebufferSize(window, &windowState.width, &windowState.height);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }
    GLint samples = 0;
    glGetIntegerv(GL_SAMPLES, &samples);
    std::cout << "OpenGL " << glGetString(GL_VERSION) << " | " << glGetString(GL_RENDERER) << " | MSAA x" << samples << "\n";

    int exitCode = 0;
    {
        std::unique_ptr<cosmos::render::Renderer> renderer = cosmos::render::createGlRenderer();
        cosmos::render::RendererConfig config;
        config.shaderDir = "shaders";
        config.debugOutput = debugContext;
        if (!renderer->init(config)) {
            std::cerr << "Renderer initialisation failed, ABORTING.\n";
            glfwDestroyWindow(window);
            return -1;
        }
        renderer->resize(windowState.width, windowState.height);

        SimLogic sim;
        sim.ResetChrono();

        // Old toy scene: a black core with dust rings
        const int DUST = 350;
        const double M_CORE = DEMO_CORE_MASS;
        const double RINGS[7] = {1.5, 4.5, 7.5, 11.0, 15.0, 19.5, 25.0};
        sim.InitBody(0, 0, M_CORE, 1.2, BodyKind::Star, 0.0f, 0.0f, 0.0f, 0, 0);
        for (int i = 0; i < DUST; ++i) {
            double jitter = (std::fmod(i * 0.6180339887, 1.0) - 0.5) * 1.2;
            double r = RINGS[i % 7] + jitter;
            double th = i * 2.39996323;
            double v = std::sqrt(M_CORE * r * r / std::pow(r * r + 0.01, 1.5));
            float t = (float)((r - 3.0) / 24.0);
            sim.InitBody(r * std::cos(th), r * std::sin(th), 1e-6, 0.08, BodyKind::Star,
                         1.0f - 0.4f * t, 0.85f - 0.1f * t, 0.6f + 0.4f * t, -v * std::sin(th), v * std::cos(th));
        }
        sim.ZeroMomentum();

        const cosmos::app::ShuttlePalette palette(2024);
        cosmos::app::RenderFrame frame;
        frame.clearColour[0] = 0.02f; frame.clearColour[1] = 0.022f; frame.clearColour[2] = 0.035f;
        frame.metresPerPixel = opt.mpp > 0.0f ? opt.mpp : DEMO_METRES_PER_PIXEL;
        frame.shuttleLengthMetres = 0.2f;   // 2 m real shuttles, scaled like the toy world; the 8 px minimum applies

        int framesDone = 0;
        double lastReport = glfwGetTime();
        int framesAtReport = 0;
        std::vector<std::uint8_t> pixels;

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) break;
            if (windowState.dirty) {
                renderer->resize(windowState.width, windowState.height);
                windowState.dirty = false;
            }

            sim.Chrono();
            sim.Update();
            const double now = glfwGetTime();

            if (windowState.width > 0 && windowState.height > 0) {
                frame.viewportWidth = windowState.width;
                frame.viewportHeight = windowState.height;
                frame.time = now;
                frame.clearObjects();
                addOldSimBodies(sim, frame);
                addDemoBodies(now, frame);
                addDemoShuttles(now, opt.shuttles, palette, frame);
                addDemoOverlay(frame);

                renderer->beginFrame(frame);
                renderer->drawLines(cosmos::render::LinePass::Underlay);
                renderer->drawBodies();
                renderer->drawShuttles();
                renderer->drawLines(cosmos::render::LinePass::Overlay);
                renderer->endFrame();

                ++framesDone;
                const bool lastFrame = opt.frames >= 0 && framesDone >= opt.frames;
                if (lastFrame && !opt.screenshot.empty()) {
                    int w = 0, h = 0;
                    if (renderer->readBackBuffer(pixels, w, h) && cosmos::app::writeBmp24(opt.screenshot, w, h, pixels.data()))
                        std::cout << "Wrote " << opt.screenshot << " (" << w << "x" << h << ")\n";
                    else { std::cerr << "Screenshot failed\n"; exitCode = 1; }
                }
                glfwSwapBuffers(window);
                if (lastFrame) break;
            }

            if (now - lastReport >= 1.0) {
                std::cout << "fps " << (framesDone - framesAtReport) / (now - lastReport) << "\n";
                lastReport = now;
                framesAtReport = framesDone;
            }
            glfwPollEvents();
        }

        const cosmos::render::RenderStats& st = renderer->frameStats();
        const cosmos::render::RenderTotals& tot = renderer->totals();
        std::cout << "Last frame: draw calls " << st.drawCalls << " (bodies " << st.bodyDrawCalls << " for " << st.bodyInstances
                  << " instances, shuttles " << st.shuttleDrawCalls << " for " << st.shuttleInstances << " instances, lines "
                  << st.lineDrawCalls << " for " << st.lineSegments << " segments), lights " << st.lightCount << ", streamed "
                  << st.streamedBytes << " bytes\n";
        std::cout << "Totals: frames " << tot.frames << ", GL errors " << tot.glErrors << ", GL warnings " << tot.glWarnings
                  << ", fence stalls " << tot.fenceStalls << ", buffer growths " << tot.arenaGrowths << "\n";
        if (tot.glErrors > 0) exitCode = 1;
        renderer->shutdown();
    }   // renderer destroyed while the context is still current

    glfwDestroyWindow(window);
    return exitCode;
}

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 2;
    if (!glfwInit()) return -1;
    const int rc = run(opt);
    glfwTerminate();
    return rc;
}
