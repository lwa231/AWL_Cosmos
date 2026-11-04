#pragma once
// Renderer interface (plan 9.1). Everything OpenGL hides behind it: core, app and ui never call GL.
//
// Per-frame use:
//     renderer.beginFrame(frame);          // uploads the frame description, clears, binds
//     renderer.drawGravityWells();         // plan 9.4 step 2 (no-op when frame.wells is empty)
//     renderer.drawAtmospheres();          // step 3 (no-op when frame.atmospheres is empty)
//     renderer.drawLines(LinePass::Underlay);   // step 4
//     renderer.drawCircles();              // step 5: Hill / SOI / Roche (no-op when frame.circles is empty)
//     renderer.drawBodies();
//     renderer.drawShuttles();
//     renderer.drawExplosions();           // fragments and sparks, additive (plan 9.4 step 8)
//     renderer.drawLines(LinePass::Overlay);
//     renderer.endFrame();                 // puts a fence behind this frame's streamed data
//     (the caller swaps buffers)
// `frame` must stay unchanged and alive until endFrame() returns.
//
// Coordinate/projection convention (documented here, used by every shader): the frame uniform buffer holds an
// orthographic projection from CENTRE-ORIGIN PIXELS (x right, y up, camera at 0,0) to clip space, plus
// metresPerPixel. Vertex shaders convert camera-relative metres to pixels first (metres / metresPerPixel) and then
// apply the projection, so the large numbers of a 1e11 m scene never reach the projection matrix.
#include "app/render_frame.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cosmos {
namespace render {

struct RendererConfig {
    std::string shaderDir = "shaders";   // folder holding the .vert/.frag/.glsl files (next to the exe)
    bool debugOutput = false;            // install glDebugMessageCallback (needs a GL debug context from the window)
};

// Per-frame counters, valid after endFrame() until the next beginFrame(). Used to verify draw-call counts.
struct RenderStats {
    std::uint32_t drawCalls = 0;           // all glDraw* calls of the frame
    std::uint32_t bodyDrawCalls = 0;
    std::uint32_t shuttleDrawCalls = 0;
    std::uint32_t lineDrawCalls = 0;
    std::uint32_t bodyInstances = 0;
    std::uint32_t lightCount = 0;
    std::uint32_t shuttleInstances = 0;
    std::uint32_t lineSegments = 0;        // quads drawn (strips + loose segments), both passes
    std::uint32_t wellSources = 0;         // bodies the gravity-well pass summed over (0: pass skipped)
    std::uint32_t atmosphereInstances = 0;
    std::uint32_t circleInstances = 0;
    std::uint32_t explosionDrawCalls = 0;  // fragment + spark draws (compute passes not counted)
    std::uint32_t explosionFragments = 0;  // fragment pool slots processed this frame
    std::uint64_t streamedBytes = 0;       // bytes written into the persistent mapped buffer this frame
};

// Cumulative counters since init().
struct RenderTotals {
    std::uint64_t frames = 0;
    std::uint64_t glErrors = 0;            // KHR_debug messages of type error / undefined behaviour
    std::uint64_t glWarnings = 0;          // other non-notification KHR_debug messages
    std::uint32_t fenceStalls = 0;         // frames that had to wait for the GPU before reusing a streaming segment
    std::uint32_t arenaGrowths = 0;        // times the streaming buffer was reallocated bigger
    // GPU time of the gravity-well pass (GL_TIME_ELAPSED query read back one or two frames late, never stalling)
    double wellsGpuMsLast = 0.0;
    double wellsGpuMsSum = 0.0;
    std::uint64_t wellsGpuSamples = 0;
    // GPU span of a whole frame (gpuFrameBegin .. gpuFrameEnd: two GL_TIMESTAMP queries), read back a few frames late
    double frameGpuMsLast = 0.0;
    double frameGpuMsSum = 0.0;
    std::uint64_t frameGpuSamples = 0;
};

enum class LinePass { Underlay, Overlay };

class Renderer {
public:
    virtual ~Renderer() = default;

    virtual bool init(const RendererConfig& config) = 0;   // needs a current GL 4.5 core context
    virtual void shutdown() = 0;                            // frees every GL object; safe to call twice
    virtual void resize(int widthPx, int heightPx) = 0;

    virtual void beginFrame(const app::RenderFrame& frame) = 0;
    virtual void drawGravityWells() = 0;                    // plan 9.4 step 2: full-screen potential bands + depth tint
    virtual void drawAtmospheres() = 0;                     // plan 9.4 step 3: radial gradient discs from R to R + top
    virtual void drawCircles() = 0;                         // plan 9.4 step 5: Hill / SOI / Roche outlines
    virtual void drawBodies() = 0;                          // plan 9.4 step 6
    virtual void drawShuttles() = 0;                        // plan 9.4 step 7 (hulls + flames, one draw call)
    virtual void drawLines(LinePass pass) = 0;              // streamed line batch (underlay: step 4/5, overlay: step 9)
    // Plan 9.4 step 8, after drawShuttles: integrates the explosion fragments (compute) and draws fragments + sparks additively.
    // Reads frame.explosions; does nothing without a live explosion. Call once per frame.
    virtual void drawExplosions() {}
    virtual void endFrame() = 0;

    // Developer aid (release benchmark): bracket the GPU work of a whole frame (call gpuFrameBegin before beginFrame and
    // gpuFrameEnd after the last draw, the UI included, before the buffer swap). The result of a frame is read back a few
    // frames later without stalling (RenderTotals::frameGpuMsLast); a frame whose slot is still in flight is not timed.
    virtual void gpuFrameBegin() {}
    virtual void gpuFrameEnd() {}

    virtual const RenderStats& frameStats() const = 0;
    virtual const RenderTotals& totals() const = 0;

    // Developer aid: copy the back buffer (call after endFrame, before swapping) as tightly packed RGB, top row first.
    virtual bool readBackBuffer(std::vector<std::uint8_t>& rgb, int& width, int& height) = 0;
};

// The OpenGL 4.5 core implementation.
std::unique_ptr<Renderer> createGlRenderer();

} // namespace render
} // namespace cosmos
