#version 450 core
// Hill / SOI / Roche circles (plan 9.4 step 5): one instanced quad per circle, the outline is carved in the fragment shader
// with a constant pixel width. (The plan names a unit-circle mesh; an impostor needs no tessellation, and a radius of
// millions of pixels stays exact because the distance comes from gl_FragCoord.)
#include "common.glsl"

struct Circle {
    float x, y;       // camera-relative centre [m]
    float radius;     // [m]
    uint  kind;       // 0 Hill, 1 SOI, 2 Roche
};
layout(std430, binding = 6) readonly buffer CircleBuffer { Circle circles[]; };

layout(location = 0) flat out vec2  vCentre;
layout(location = 1) flat out float vRadiusPx;
layout(location = 2) flat out uint  vKind;
layout(location = 3) flat out float vDashes;     // dashes around the whole circle (a whole number: the pattern closes without a seam)

const float MIN_RADIUS_PX = 3.0;       // a smaller circle is a smudge on its body
const float MAX_RADIUS_PX = 5.0e6;     // beyond this a float cannot place the rim to a pixel; it is a straight edge off-screen anyway
const float MARGIN = 3.0;

void main()
{
    Circle c = circles[gl_InstanceID];
    vec2 corner01 = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));

    float radiusPx = c.radius * uScale.y;
    vec2 centrePx = vec2(c.x, c.y) * uScale.y;
    vec2 viewHalf = 0.5 * uViewport.xy + MARGIN;

    // Distance from the centre to the nearest and the farthest point of the viewport: the rim crosses the screen only between them
    float dMin = length(clamp(centrePx, -viewHalf, viewHalf) - centrePx);
    float dMax = length(max(abs(centrePx - viewHalf), abs(centrePx + viewHalf)));
    bool visible = radiusPx >= MIN_RADIUS_PX && radiusPx <= MAX_RADIUS_PX &&
                   radiusPx + MARGIN >= dMin && radiusPx - MARGIN <= dMax;

    float halfSize = radiusPx + MARGIN;
    vec2 lo = max(centrePx - halfSize, -viewHalf);
    vec2 hi = min(centrePx + halfSize, viewHalf);
    if (!visible || lo.x > hi.x || lo.y > hi.y) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    } else {
        gl_Position = uProj * vec4(mix(lo, hi, corner01), 0.0, 1.0);
    }

    float period = c.kind == 1u ? 14.0 : 7.0;          // pixels per dash (SOI) or dot (Roche); unused for Hill
    vCentre = centrePx;
    vRadiusPx = radiusPx;
    vKind = c.kind;
    vDashes = max(floor(6.2831853 * radiusPx / period + 0.5), 1.0);
}
