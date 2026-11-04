#version 450 core
// Gravity-well rings (plan 9.5): Phi(x) = -sum mu_j / max(|x - x_j|, R_j), u = log(-Phi), bands at integer values of
// u * k, drawn as anti-aliased lines through fwidth. The background tint ramps with u (deeper = brighter).
//
// Precision: x is the pixel's CAMERA-RELATIVE position in metres (pixel * metresPerPixel) and x_j is the body's
// camera-relative position, so the difference is a float subtraction of two numbers of similar size even at 1e11 m. The
// loop is one inversesqrt and one fma per source: 64 sources at 1080p is ~130M iterations.
#include "common.glsl"

struct Well {
    float x, y;       // camera-relative position [m]
    float radius;     // inside it the potential is flat
    float mu;         // G m
};
layout(std430, binding = 4) readonly buffer WellBuffer { Well wells[]; };

layout(location = 0) uniform vec4 uStyle;    // x: bands per unit of u, y: ramp low, z: ramp high, w: line alpha
layout(location = 1) uniform vec3 uBack;     // the frame's clear colour
layout(location = 2) uniform vec3 uTint;     // colour added at full depth
layout(location = 3) uniform vec3 uLine;     // band line colour

layout(location = 0) out vec4 fragColour;

void main()
{
    vec2 p = (gl_FragCoord.xy - 0.5 * uViewport.xy) * uScale.x;   // camera-relative metres

    float phi = 0.0;                                              // -Phi, positive
    for (uint i = 0u; i < uCounts.w; ++i) {
        Well w = wells[i];
        vec2 d = p - vec2(w.x, w.y);
        float r2 = max(dot(d, d), w.radius * w.radius);
        phi += w.mu * inversesqrt(r2);
    }

    float u = log(max(phi, 1e-30));
    float f = u * uStyle.x;
    float fw = max(fwidth(f), 1e-6);                              // 0 inside a flat core: the lines are then either all or nothing

    float t = smoothstep(uStyle.y, uStyle.z, u);                  // depth 0..1
    vec3 back = uBack + uTint * t;

    float frac = fract(f);
    float distPx = min(frac, 1.0 - frac) / fw;                    // distance to the nearest band edge in pixels
    float cover = clamp(1.25 - distPx, 0.0, 1.0);                 // ~1 px wide with a soft edge
    float fade = 1.0 - smoothstep(0.25, 0.5, fw);                 // bands closer than ~2-4 px melt into the tint (no moire)
    float a = uStyle.w * (0.5 + 0.5 * t) * cover * fade;

    fragColour = vec4(mix(back, uLine, a), 1.0);                  // opaque: this pass replaces the clear colour
}
