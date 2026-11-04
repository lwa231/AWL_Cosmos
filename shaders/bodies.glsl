// Body and light storage buffers (runtime counts, no MAX_LIGHTS). Plain floats keep std430 layout trivial.
// Must match BodyGpu / LightGpu in gl_renderer.cpp (32 bytes each).
struct Body {
    float x, y;          // camera-relative position [m]
    float radius;        // [m]
    float ambient;
    float r, g, b;       // colour (sRGB encoded)
    uint  flags;         // bit 0: flat (emissive or unlit), bit 1: black hole
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
