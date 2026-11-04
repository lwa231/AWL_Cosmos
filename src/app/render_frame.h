#pragma once
// Plain-data description of one rendered frame (plan 9.1-9.3). Header-only and GL-free: the app (M4) fills it, the
// Renderer (src/render) consumes it. Nothing here knows about OpenGL.
//
// Conventions
//  - Positions are CAMERA-RELATIVE metres in the view plane: x right, y up, camera at (0,0). The app subtracts the
//    camera from the world position in double and only then casts to float (plan 9.3), so there is no jitter at 1e11 m.
//  - metresPerPixel is exp(zoomLog), owned by the camera (M4). The renderer only consumes it.
//  - Colours are display-ready sRGB-encoded values in [0,1] and are written to the (non-sRGB) framebuffer as they are,
//    exactly like the original renderer did. Lighting maths is done in that same space.
//  - Draw order inside a pass is array order (no depth buffer): later entries draw over earlier ones.
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cosmos {
namespace app {

struct RenderColour {
    float r = 1.0f, g = 1.0f, b = 1.0f;
};

// A sphere impostor (a lit disc). Radius is the physical radius; the renderer enforces the minimum marker size.
struct RenderBody {
    float x = 0.0f, y = 0.0f;   // camera-relative position [m]
    float radius = 1.0f;        // [m]
    RenderColour colour;
    float ambient = 0.04f;      // faint light on the night side (lit bodies only)
    bool emissive = false;      // stars: drawn flat/bright, ignores lights
    bool receivesLight = true;  // false behaves like emissive (drawn flat)
    bool isBlackHole = false;   // dark disc + thin bright ring in `colour` (plan 9.6)
};

// A light source, camera-relative. Any number (runtime count, no MAX_LIGHTS).
struct RenderLight {
    float x = 0.0f, y = 0.0f;   // camera-relative position [m]
    RenderColour colour;        // light colour
    float intensity = 1.0f;     // multiplier on colour
};

// One shuttle: a triangle of 3 coloured sectors (plan 7.4) plus a flame behind the nozzle.
struct RenderShuttle {
    float x = 0.0f, y = 0.0f;   // camera-relative position [m]
    float heading = 0.0f;       // rad, counter-clockwise from +x; the nose points this way
    RenderColour sector[3];     // the three sector colours (palette triple)
    float throttle = 0.0f;      // 0..1: flame length and brightness
    float flickerPhase = 0.0f;  // rad: per-shuttle phase so flames do not flicker in unison (visual only)
};

// Overlay geometry (plan 9.5), camera-relative metres with per-vertex RGBA (straight, not premultiplied alpha).
struct LineVertex {
    float x = 0.0f, y = 0.0f;
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;
};

struct LineStrip {          // `count` consecutive LineBatch::vertices starting at `first`
    std::uint32_t first = 0;
    std::uint32_t count = 0;
    float widthPx = 1.0f;   // on-screen width in pixels (not metres)
};

struct LineSegment {
    LineVertex a, b;
    float widthPx = 1.0f;
};

// A batch of polylines and loose segments drawn in one pass. Colours interpolate along each segment.
struct LineBatch {
    std::vector<LineVertex> vertices;   // storage for the strips
    std::vector<LineStrip> strips;
    std::vector<LineSegment> segments;

    void clear() { vertices.clear(); strips.clear(); segments.clear(); }
    bool empty() const { return strips.empty() && segments.empty(); }

    void addStrip(const LineVertex* v, std::size_t n, float widthPx)
    {
        if (n < 2) return;
        strips.push_back({static_cast<std::uint32_t>(vertices.size()), static_cast<std::uint32_t>(n), widthPx});
        vertices.insert(vertices.end(), v, v + n);
    }
    void addStrip(const std::vector<LineVertex>& v, float widthPx) { addStrip(v.data(), v.size(), widthPx); }
    void addSegment(const LineVertex& a, const LineVertex& b, float widthPx) { segments.push_back({a, b, widthPx}); }

    // Number of quads the renderer will draw for this batch.
    std::size_t segmentCount() const
    {
        std::size_t n = segments.size();
        for (const LineStrip& s : strips) n += s.count > 1 ? s.count - 1 : 0;
        return n;
    }
};

struct RenderFrame {
    int viewportWidth = 0;      // framebuffer size in pixels
    int viewportHeight = 0;
    float metresPerPixel = 1.0f;
    double time = 0.0;          // real seconds, for visual animation only (flame flicker)
    float clearColour[3] = {0.0f, 0.0f, 0.02f};

    // Minimum on-screen sizes (plan 9.3, 9.6). A real shuttle is ~2 m: invisible without them.
    float bodyMinPixels = 3.0f;         // a body smaller than this (diameter) is drawn as a marker of this size
    float shuttleLengthMetres = 2.0f;   // physical hull length
    float shuttleMinPixels = 8.0f;      // hull length is never drawn shorter than this

    std::vector<RenderBody> bodies;
    std::vector<RenderLight> lights;
    std::vector<RenderShuttle> shuttles;
    LineBatch underlay;         // drawn before bodies (trails, orbits, Hill/SOI circles; plan 9.4 steps 4-5)
    LineBatch overlay;          // drawn after shuttles (velocity/acceleration arrows; plan 9.4 step 9)

    // Empties every array but keeps the capacity, so the per-frame fill does not reallocate.
    void clearObjects()
    {
        bodies.clear();
        lights.clear();
        shuttles.clear();
        underlay.clear();
        overlay.clear();
    }
};

} // namespace app
} // namespace cosmos
