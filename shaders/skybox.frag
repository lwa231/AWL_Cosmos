#version 450 core
// Procedural sky (V1.1 step 3): faint white, yellow, red, green and blue specks plus a subtle nebula haze. The sky is at
// infinity, so in this top-down orthographic view it is fixed to the screen: panning and zooming do not move it (no
// parallax). Seeded by uVisual.z, so every run shows the same sky. Additive (alpha 0) over the clear colour or the
// gravity-well background. uVisual.y: 0 Low (one sparse layer), 1 Medium (two layers, nebula), 2 High (three layers, a
// richer nebula).
#include "common.glsl"
#include "noise.glsl"

layout(location = 0) out vec4 fragColour;

vec3 speckColour(float h)
{
    if (h < 0.40) return vec3(1.0, 1.0, 1.0);        // white
    if (h < 0.62) return vec3(1.0, 0.92, 0.65);      // yellow
    if (h < 0.77) return vec3(1.0, 0.62, 0.50);      // red
    if (h < 0.92) return vec3(0.65, 0.78, 1.0);      // blue
    return vec3(0.65, 1.0, 0.72);                    // green
}

// One layer of specks: at most one per `cell` x `cell` pixel cell, with probability `density`
vec3 speckLayer(vec2 px, float cell, float density, float peak, uint salt)
{
    vec2 g = px / cell;
    ivec2 ci = ivec2(floor(g));
    uint h = hashU(hash2u(ci) ^ salt);
    if (u2f(h) >= density) return vec3(0.0);
    uint h1 = hashU(h), h2 = hashU(h1), h3 = hashU(h2), h4 = hashU(h3);
    vec2 centre = (vec2(ci) + 0.2 + 0.6 * vec2(u2f(h1), u2f(h2))) * cell;
    float b = u2f(h3);
    float bright = peak * (0.15 + 0.85 * b * b * b);
    vec2 dd = px - centre;
    float w = exp(-dot(dd, dd) * 1.6);               // ~1 px speck
    return speckColour(u2f(h4)) * bright * w;
}

void main()
{
    float q = uVisual.y;
    vec2 px = gl_FragCoord.xy - 0.5 * uViewport.xy;  // centre-origin: a resize keeps the middle of the sky in place
    uint seed = uint(uVisual.z * 16777216.0);
    vec3 col = speckLayer(px, 9.0, 0.16, 0.42, seed ^ 0x1234567u);
    if (q >= 1.0) col += speckLayer(px + vec2(3.5, 1.5), 23.0, 0.10, 0.85, seed ^ 0x89abcdefu);
    if (q >= 2.0) col += speckLayer(px + vec2(1.5, 4.5), 5.0, 0.22, 0.18, seed ^ 0x2468aceu);

    if (q >= 1.0) {
        int oct = q >= 2.0 ? 6 : 4;
        vec2 s = px / 520.0 + vec2(uVisual.z * 97.0, uVisual.z * 31.0);
        float n = fbm2(s, oct);
        float hue = fbm2(s * 0.6 + vec2(5.2, 1.3), 3);
        vec3 haze = mix(vec3(0.30, 0.10, 0.42), vec3(0.05, 0.25, 0.38), smoothstep(0.35, 0.65, hue));
        haze = mix(haze, vec3(0.45, 0.15, 0.20), smoothstep(0.6, 0.8, fbm2(s * 1.7 + vec2(9.1), 2)) * 0.5);
        float dens = smoothstep(0.45, 0.85, n);
        col += haze * dens * 0.075;
        col *= 1.0 + dens * 0.8;                      // more specks show through the haze
    }
    fragColour = vec4(col, 0.0);
}
