#version 450 core
// Fragments and their sparks (plan 9.7), additive. One instanced quad per (fragment, k): k = 0 is the fragment itself, k = 1..8
// are its sparks. Sparks are visual children animated by a formula: they trail behind the fragment along its velocity (a lag
// of k/8 of a fixed sim time), scatter a few pixels sideways, twinkle, and run the colour ramp white - yellow - orange - red
// as the fragment ages (u = min(simAge / life, realAge / minReal), the same fade the compute shader and the CPU use).
// Fragment positions are relative to the explosion's anchor, whose camera-relative offset is in the explosion table.
#include "common.glsl"

struct Fragment {
    float px, py, vx, vy;
    float simAge, realAge, life, minReal;
    float ballistic, size;
    uint slot, serial, landed, seed;
    float pad0, pad1;
};
struct Body {
    float x, y, mu, radius;
    float indX, indY, vx, vy;
    float rho0, scaleHeight, spin, altitude;
};
struct Explosion {
    float anchorX, anchorY, dtSim, dtReal;
    uint serial, bodyCount, pad0, pad1;
    Body bodies[4];
};
layout(std430, binding = 8) readonly buffer FragmentBuffer { Fragment frags[]; };
layout(std430, binding = 9) readonly buffer ExplosionBuffer { Explosion explosions[]; };

layout(location = 0) out vec2 vCorner;
layout(location = 1) out vec4 vColour;   // rgb colour, a = coverage multiplier

const uint PER = 9u;                      // 1 fragment + 8 sparks
const float TRAIL_SECONDS = 0.30;         // sim seconds of the longest spark lag
const float MAX_TRAIL_PX = 60.0;

vec3 ramp(float t)
{
    const vec3 white = vec3(1.0, 0.97, 0.90), yellow = vec3(1.0, 0.85, 0.25), orange = vec3(1.0, 0.45, 0.08), red = vec3(0.65, 0.08, 0.02);
    t = clamp(t, 0.0, 1.0);
    if (t < 0.33) return mix(white, yellow, t / 0.33);
    if (t < 0.66) return mix(yellow, orange, (t - 0.33) / 0.33);
    return mix(orange, red, (t - 0.66) / 0.34);
}

uint pcg(uint x)
{
    uint state = x * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}
float rand01(uint x) { return float(pcg(x) & 0xFFFFFFu) / 16777216.0; }

void cull()
{
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    vCorner = vec2(0.0);
    vColour = vec4(0.0);
}

void main()
{
    uint fi = uint(gl_InstanceID) / PER;
    uint k = uint(gl_InstanceID) % PER;
    Fragment f = frags[fi];
    if (f.serial == 0u || f.slot >= 64u || explosions[f.slot].serial != f.serial) { cull(); return; }
    float u = min(f.simAge / f.life, f.realAge / f.minReal);
    if (u >= 1.0) { cull(); return; }

    Explosion e = explosions[f.slot];
    vec2 centrePx = (vec2(e.anchorX, e.anchorY) + vec2(f.px, f.py)) * uScale.y;
    float fade = 1.0 - smoothstep(0.65, 1.0, u);
    float halfSize;
    vec3 colour;
    float alpha;

    if (k == 0u) {   // the fragment: a hot speck that cools
        halfSize = max(1.0, 1.1 * f.size);
        colour = ramp(u * 0.75);
        alpha = fade * 0.55;
    } else {
        float s = float(k) / 8.0;
        uint h = f.seed + k * 2654435761u;
        // trail behind the motion, in pixels, capped; embers at rest have none and glow in place
        vec2 trail = -vec2(f.vx, f.vy) * (TRAIL_SECONDS * (1.0 - u) * s) * uScale.y;
        float tl = length(trail);
        if (tl > MAX_TRAIL_PX) trail *= MAX_TRAIL_PX / tl;
        vec2 jitter = (vec2(rand01(h), rand01(h ^ 0x9E3779B9u)) * 2.0 - 1.0) * (3.0 * s * (1.0 - u) + 0.5);
        centrePx += trail + jitter;
        float twinkle = 0.65 + 0.35 * sin(uScale.z * (18.0 + 20.0 * rand01(h ^ 0x85EBCA6Bu)) + 6.2831853 * rand01(h));
        halfSize = 0.9 + 0.4 * f.size * (1.0 - s);
        colour = ramp(u * 0.85 + s * 0.3);
        alpha = fade * (1.0 - s) * 0.3 * twinkle;
    }

    vec2 corner = vec2(float(gl_VertexID & 1) * 2.0 - 1.0, float(gl_VertexID >> 1) * 2.0 - 1.0);
    gl_Position = uProj * vec4(centrePx + corner * (halfSize + 0.75), 0.0, 1.0);
    vCorner = corner * (halfSize + 0.75) / halfSize;
    vColour = vec4(colour, alpha);
}
