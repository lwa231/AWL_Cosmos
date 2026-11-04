#version 450 core
// Fragments and their sparks (plan 9.7), additive. One instanced quad per (fragment, k): k = 0 is the fragment itself, k = 1..8
// are its sparks. Sparks are visual children animated by a formula: they trail behind the fragment along its velocity (a lag
// of k/8 of a fixed sim time), scatter a few pixels sideways, twinkle, and run the colour ramp white - yellow - orange - red
// as the fragment ages (u = min(simAge / life, realAge / minReal), the same fade the compute shader and the CPU use).
// Fragment positions are relative to the explosion's anchor, whose camera-relative offset is in the explosion table.
//
// Minimum cloud size (plan 9.3, the same idea as the minimum body and hull sizes): a 100 m cloud seen at 2 km/px would be a
// single pixel. In this vertex stage only (the simulation is untouched) the offsets of the fragments from the contact point on the
// ground are magnified so that the typical fragment sits about MIN_CLOUD_PX pixels from it; the magnification is 1 as soon as the
// cloud is that big on its own. The typical distance comes from the compute pass (Stats buffer). The cloud grows to that size
// over the first moments of REAL time, so the burst reads as a burst, and moves with sim time like the real one (landed
// fragments turn with the ground), so after a warp resume the cloud visibly sweeps along.
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
    uint serial, bodyCount, ground, pad1;
    Body bodies[4];
};
layout(std430, binding = 8) readonly buffer FragmentBuffer { Fragment frags[]; };
layout(std430, binding = 9) readonly buffer ExplosionBuffer { Explosion explosions[]; };

layout(std430, binding = 10) readonly buffer StatsBuffer { uint statRoot[64]; uint statCount[64]; };

layout(location = 0) out vec2 vCorner;
layout(location = 1) out vec4 vColour;   // rgb colour, a = coverage multiplier

const uint PER = 9u;                      // 1 fragment + 8 sparks
const float TRAIL_SECONDS = 0.30;         // sim seconds of the longest spark lag
const float MAX_TRAIL_PX = 60.0;
const float MIN_CLOUD_PX = 34.0;          // typical fragment distance from the contact point on screen, at least (cloud ~ 3x that wide)
const float MAX_CLOUD_PX = 130.0;         // magnified offsets saturate smoothly towards this
const float GROW_REAL_S = 0.5;            // real seconds the minimum-size cloud takes to open up
const float MAX_MAG = 1.0e5;

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

// The contact point on the turning ground, relative to the anchor (see fragments.comp).
vec2 groundPoint(Explosion e, float age)
{
    if (e.ground == 0u || e.bodyCount == 0u) return vec2(0.0);
    Body b = e.bodies[0];
    float th = b.spin * age;
    float s2 = sin(0.5 * th);
    return (2.0 * s2 * s2) * vec2(b.x, b.y) + sin(th) * vec2(b.y, -b.x);
}

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
    float px = uScale.y;   // pixels per metre

    // Render-time magnification about the contact point (1 when the cloud is big enough on screen by itself)
    float cnt = float(statCount[f.slot]);
    float rootMean = cnt > 0.0 ? float(statRoot[f.slot]) / (16.0 * cnt) : 0.0;
    float typPx = rootMean * rootMean * px;                                   // typical offset of a fragment, pixels
    float want = MIN_CLOUD_PX * clamp(0.08 + f.realAge / GROW_REAL_S, 0.0, 1.0);
    float mag = typPx < want ? min(want / max(typPx, 1.0e-6 * want), MAX_MAG) : 1.0;
    vec2 ground = groundPoint(e, f.simAge);
    vec2 off = (vec2(f.px, f.py) - ground) * px;                              // true offset, pixels
    vec2 offV = off;
    float typDisp = typPx;
    if (mag > 1.0) {
        offV = off * mag;
        float len = length(offV);
        if (len > 1.0e-3) offV *= MAX_CLOUD_PX * (1.0 - exp(-len / MAX_CLOUD_PX)) / len;   // saturate, never fly off the screen
        typDisp = want;
    }
    vec2 centrePx = (vec2(e.anchorX, e.anchorY) + ground) * px + offV;
    float distPx = length(offV);

    float fade = 1.0 - smoothstep(0.65, 1.0, u);
    // Many overlapping additive points would burn to white: scale the brightness by how crowded the cloud is
    float crowd = clamp(1.8 * 3.14159 * typDisp * typDisp / max(cnt * float(PER) * 3.0, 1.0), 0.04, 1.0);
    float heat = smoothstep(0.0, 2.5 * max(typDisp, 1.0), distPx);            // hot core, cooler rim
    float halfSize;
    vec3 colour;
    float alpha;

    if (k == 0u) {   // the fragment: a hot speck that cools
        halfSize = max(1.5, 1.1 * f.size);
        colour = ramp(u * 0.75 + 0.25 * heat);
        alpha = fade * 0.7 * crowd;
    } else {
        float s = float(k) / 8.0;
        uint h = f.seed + k * 2654435761u;
        // trail behind the motion, in pixels, capped; embers at rest have none and glow in place
        vec2 trail = -vec2(f.vx, f.vy) * (TRAIL_SECONDS * (1.0 - u) * s) * px * (mag > 1.0 ? min(mag, 8.0) : 1.0);
        float tl = length(trail);
        if (tl > MAX_TRAIL_PX) trail *= MAX_TRAIL_PX / tl;
        float spread = 3.0 * s * (1.0 - u) + 0.5 + (mag > 1.0 ? 0.35 * typDisp * s : 0.0);
        vec2 jitter = (vec2(rand01(h), rand01(h ^ 0x9E3779B9u)) * 2.0 - 1.0) * spread;
        // Magnified clouds of fragments that rest on the ground are a line along the surface: the sparks rise off it
        // (away from the body's centre), so the cloud has a body, and the further sparks of a fragment rise higher
        vec2 rise = vec2(0.0);
        if (mag > 1.0 && e.ground != 0u && e.bodyCount > 0u) {
            vec2 out2 = vec2(f.px, f.py) - vec2(e.bodies[0].x, e.bodies[0].y);
            float ol = length(out2);
            if (ol > 0.0) rise = out2 / ol * (typDisp * 1.3 * s * pow(rand01(h ^ 0x27D4EB2Fu), 1.5));
        }
        centrePx += trail + jitter + rise;
        float twinkle = 0.65 + 0.35 * sin(uScale.z * (18.0 + 20.0 * rand01(h ^ 0x85EBCA6Bu)) + 6.2831853 * rand01(h));
        halfSize = max(1.2, 0.9 + 0.4 * f.size * (1.0 - s));
        colour = ramp(u * 0.85 + s * 0.3 * (1.0 - u) + 0.25 * heat);
        alpha = fade * (1.0 - s) * 0.6 * twinkle * crowd;
    }

    vec2 corner = vec2(float(gl_VertexID & 1) * 2.0 - 1.0, float(gl_VertexID >> 1) * 2.0 - 1.0);
    gl_Position = uProj * vec4(centrePx + corner * (halfSize + 0.75), 0.0, 1.0);
    vCorner = corner * (halfSize + 0.75) / halfSize;
    vColour = vec4(colour, alpha);
}
