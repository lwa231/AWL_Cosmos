#version 450 core
// All shuttles in ONE draw call of 2N instances x 6 vertices. Instances 0..N-1 are the flame quads (2 triangles) and
// instances N..2N-1 the hull triangles (vertices 3-5 collapse onto vertex 0), so every flame is drawn before every hull:
// an additive flame never lands on top of another shuttle's opaque hull. Instance data is pulled from a storage buffer.
#include "common.glsl"

struct Shuttle {
    vec4 pose;     // x, y (camera-relative metres), heading (rad), throttle
    vec4 colour0;  // sector 0 rgb, flicker phase
    vec4 colour1;  // sector 1 rgb, unused
    vec4 colour2;  // sector 2 rgb, unused
};
layout(std430, binding = 2) readonly buffer ShuttleBuffer { Shuttle shuttles[]; };

layout(location = 0) out vec3 vBary;                  // hull: barycentric coordinate (selects the sector)
layout(location = 1) out vec2 vFlame;                 // flame: x across (-1..1), y along (0 nozzle .. 1 tip)
layout(location = 2) flat out int  vKind;             // 0 flame, 1 hull
layout(location = 3) flat out vec4 vColourA;          // hull: sector 0 rgb, flame: throttle in w
layout(location = 4) flat out vec3 vColourB;
layout(location = 5) flat out vec3 vColourC;
layout(location = 6) flat out float vFlicker;         // flame brightness flicker multiplier

// Hull in units of the hull length, nose at +x. Nozzle at the middle of the rear edge.
const vec2 HULL[3] = vec2[3](vec2(0.5, 0.0), vec2(-0.5, 0.35), vec2(-0.5, -0.35));
const vec2 NOZZLE = vec2(-0.5, 0.0);
const int  FLAME_TRI[6] = int[6](0, 1, 2, 2, 1, 3);   // quad corners 0..3 as two triangles

void main()
{
    const int count = int(uCounts.z);
    const bool isHull = gl_InstanceID >= count;
    Shuttle s = shuttles[isHull ? gl_InstanceID - count : gl_InstanceID];
    float throttle = clamp(s.pose.w, 0.0, 1.0);
    float phase = s.colour0.w;

    float lengthPx = max(uShuttle.x * uScale.y, uShuttle.y);
    vec2 centrePx = s.pose.xy * uScale.y;
    float c = cos(s.pose.z), sn = sin(s.pose.z);
    mat2 rot = mat2(c, sn, -sn, c);                    // columns: local +x, local +y

    // Visual-only flicker: two incommensurate sines, per-shuttle phase
    float t = uScale.z;
    float flicker = 1.0 + 0.10 * sin(t * 47.0 + phase) + 0.06 * sin(t * 113.0 + phase * 2.3);

    vec2 local;
    int v = gl_VertexID;
    if (!isHull) {
        vKind = 0;
        int corner = FLAME_TRI[v];
        float along = float(corner >> 1);              // 0 at the nozzle, 1 at the tip
        float across = float(corner & 1) * 2.0 - 1.0;  // -1 .. 1
        float flameLen = (0.25 + 2.0 * throttle) * flicker;    // in hull lengths
        float halfWidth = 0.30;
        local = NOZZLE + vec2(-along * flameLen, across * halfWidth);
        if (throttle < 0.01) local = NOZZLE;           // degenerate quad: nothing drawn at zero throttle
        vFlame = vec2(across, along);
        vBary = vec3(0.0);
    } else {
        vKind = 1;
        int i = v < 3 ? v : 0;                         // second triangle of the 6 vertices is degenerate
        local = HULL[i];
        vBary = vec3(i == 0 ? 1.0 : 0.0, i == 1 ? 1.0 : 0.0, i == 2 ? 1.0 : 0.0);
        vFlame = vec2(0.0);
    }

    vColourA = vec4(s.colour0.rgb, throttle);
    vColourB = s.colour1.rgb;
    vColourC = s.colour2.rgb;
    vFlicker = flicker;

    vec2 px = centrePx + rot * (local * lengthPx);
    gl_Position = uProj * vec4(px, 0.0, 1.0);
}
