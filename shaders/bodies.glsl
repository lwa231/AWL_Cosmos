// Body and light storage buffers (runtime counts, no MAX_LIGHTS). Plain floats keep std430 layout trivial.
// Must match BodyGpu / LightGpu in gl_renderer.cpp (80 and 32 bytes).
struct Body {
    float x, y;          // camera-relative position [m]
    float radius;        // [m]
    float ambient;
    float r, g, b;       // colour (sRGB encoded)
    uint  flags;         // bit 0: flat (emissive or unlit), bit 1: black hole
    uint  style;         // app::BodyStyle
    float temperature;   // stars [K]
    float seed;          // 0..1
    float spinPhase;     // rad
    float detail;        // 1: the body with the most detail
    float cloudCover;    // 0..1
    float atmRim;        // 0..1
    float pad0;
    float p0, p1, p2;    // planets: atmosphere tint; black holes: accretion-disk normal
    float pad1;
};

struct Light {
    float x, y;          // camera-relative position [m]
    float intensity;
    float pad0;
    float r, g, b;       // colour
    float pad1;
};

layout(std430, binding = 0) readonly buffer BodyBuffer  { Body  bodies[]; };
layout(std430, binding = 1) readonly buffer LightBuffer { Light lights[]; };

const uint BODY_FLAT       = 1u;
const uint BODY_BLACK_HOLE = 2u;

const uint STYLE_PLAIN = 0u, STYLE_STAR = 1u, STYLE_ROCKY = 2u, STYLE_OCEAN = 3u, STYLE_GAS = 4u, STYLE_ICY = 5u, STYLE_BH = 6u;

// Black-hole geometry in units of the horizon radius R (= the body radius, 2 mu / c^2)
const float BH_SHADOW = 2.6;     // apparent shadow radius (sqrt(27)/2 R)
const float BH_DISK_IN = 3.0;    // innermost stable orbit
const float BH_DISK_OUT = 11.0;
const float BH_EXTENT = 14.0;    // quad half-size
// Star halo in units of the disc radius, plus a bloom in pixels
const float STAR_CORONA = 2.2;

// Pixel unit of a black hole: its horizon radius on screen, but never so small that the hole vanishes
float bhUnitPx(float radiusPx) { return max(radiusPx, 0.5 * uScale.w / BH_SHADOW * 1.4); }
// Bloom falloff length of a star in pixels
float starBloomPx(float discPx) { return 5.0 + 0.35 * discPx; }
