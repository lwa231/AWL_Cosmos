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

// ---- M7 overlays that read the body list rather than stream lines (plan 9.5) --------------------------------------------

// One gravity source of the well-ring pass (plan 9.5): Phi(x) = -sum mu_j / max(|x - x_j|, R_j). Camera-relative metres.
struct RenderWell {
    float x = 0.0f, y = 0.0f;   // camera-relative position [m]
    float radius = 1.0f;        // R_j [m]: inside it the potential is flat (the field of an overlapped body is not 1/r)
    float mu = 0.0f;            // G m [m^3/s^2]. A float is enough: 1e20 is far inside its range and 7 digits suit a picture
};

// Look of the ring pass. u = log(-Phi) [Phi in m^2/s^2]; a band edge falls at every 1/bandsPerUnit in u.
struct WellStyle {
    float bandsPerUnit = 1.6f;    // k: lines per e-fold of -Phi
    float rampLow = 12.0f;        // u at or below which the background is the plain clear colour
    float rampHigh = 24.0f;       // u at which the depth tint is full (u = 22 is a star surface in Lucerna, 18 a planet's)
    float lineAlpha = 0.45f;      // opacity of a band line before the depth gain
    float tint[3] = {0.10f, 0.065f, 0.24f};   // colour added to the background at full depth
    float line[3] = {0.45f, 0.55f, 0.90f};   // band line colour
};

// An atmosphere bubble (plan 5.5, 9.5): a radial gradient disc from R to R + top, tinted per body.
struct RenderAtmosphere {
    float x = 0.0f, y = 0.0f;   // camera-relative centre of the body [m]
    float radius = 1.0f;        // R [m]
    float top = 0.0f;           // height at which the density reaches the 1e-6 kg/m^3 cutoff [m]; 0 = no bubble
    float scaleHeight = 1.0f;   // H [m]: density falls as exp(-h / H)
    RenderColour tint{0.45f, 0.65f, 1.0f};
};

// Hill, sphere-of-influence and Roche circles (plan 5.8, 9.5): outlines of constant pixel width, one style per kind.
enum class CircleKind : std::uint32_t { Hill = 0, Soi = 1, Roche = 2 };
struct RenderCircle {
    float x = 0.0f, y = 0.0f;   // camera-relative centre [m]
    float radius = 1.0f;        // [m]
    CircleKind kind = CircleKind::Hill;
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

    // M7 passes. Each stays empty when its toggle is off; the renderer then skips the pass.
    std::vector<RenderWell> wells;              // gravity-well rings (step 2). One entry per source body
    WellStyle wellStyle;
    std::vector<RenderAtmosphere> atmospheres;  // atmosphere bubbles (step 3)
    float atmosphereAlpha = 0.70f;              // opacity at the surface
    float atmospherePower = 0.30f;              // alpha = atmosphereAlpha * (rho / rho0)^power: a small power keeps the thin air visible
    std::vector<RenderCircle> circles;          // Hill / SOI / Roche outlines (step 5)

    // Empties every array but keeps the capacity, so the per-frame fill does not reallocate.
    void clearObjects()
    {
        bodies.clear();
        lights.clear();
        shuttles.clear();
        underlay.clear();
        overlay.clear();
        wells.clear();
        atmospheres.clear();
        circles.clear();
    }
};

} // namespace app
} // namespace cosmos
