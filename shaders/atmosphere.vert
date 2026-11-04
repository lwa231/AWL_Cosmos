#version 450 core
// Atmosphere bubbles (plan 9.4 step 3): one instanced quad per atmosphere covering the disc of radius R + top. Like the
// body impostors the quad is clamped to the viewport and the fragment shader works from gl_FragCoord.
#include "common.glsl"

struct Atmosphere {
    float x, y;        // camera-relative centre [m]
    float radius;      // R [m]
    float top;         // height of the outer edge above the surface [m]
    float scaleHeight; // H [m]
    float r, g, b;     // tint
};
layout(std430, binding = 5) readonly buffer AtmosphereBuffer { Atmosphere atmospheres[]; };

layout(location = 0) flat out vec2  vCentre;     // centre in centre-origin pixels
layout(location = 1) flat out float vRadiusPx;   // R in pixels
layout(location = 2) flat out float vTopPx;      // shell thickness in pixels
layout(location = 3) flat out uint  vAtm;

void main()
{
    Atmosphere a = atmospheres[gl_InstanceID];
    vec2 corner01 = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));

    float radiusPx = a.radius * uScale.y;
    float topPx = a.top * uScale.y;
    float outer = radiusPx + topPx + 1.0;
    vec2 centrePx = vec2(a.x, a.y) * uScale.y;

    vec2 viewHalf = 0.5 * uViewport.xy + 2.0;
    vec2 lo = max(centrePx - outer, -viewHalf);
    vec2 hi = min(centrePx + outer, viewHalf);
    if (topPx < 1.0 || lo.x > hi.x || lo.y > hi.y) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);       // a shell thinner than a pixel, or entirely off-screen: culled
    } else {
        gl_Position = uProj * vec4(mix(lo, hi, corner01), 0.0, 1.0);
    }
    vCentre = centrePx;
    vRadiusPx = radiusPx;
    vTopPx = topPx;
    vAtm = uint(gl_InstanceID);
}
