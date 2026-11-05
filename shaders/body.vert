#version 450 core
// Sphere impostors: one instanced quad per body, the disc is carved in the fragment shader. Stars get a larger quad for
// the corona and bloom, black holes one for the accretion disk.
#include "common.glsl"
#include "bodies.glsl"

layout(location = 0) flat out vec2  vCentre;       // disc centre in centre-origin pixels (same frame as gl_FragCoord - viewport/2)
layout(location = 1) flat out float vDrawRadius;   // on-screen disc radius in pixels (>= half the minimum marker size)
layout(location = 2) flat out uint  vBody;
layout(location = 3) flat out float vMarker;       // 1 when drawn as a minimum-size marker

void main()
{
    Body b = bodies[gl_InstanceID];
    vec2 corner01 = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));   // triangle strip order

    float radiusPx = b.radius * uScale.y;
    float drawRadius = max(radiusPx, 0.5 * uScale.w);
    float halfSize = drawRadius + 1.5;                      // margin for the anti-aliased edge
    if (b.style == STYLE_STAR) {
        float bloom = starBloomPx(drawRadius) * (uVisual.x >= 1.0 ? 16.0 : 6.0);
        halfSize = max(drawRadius * STAR_CORONA, drawRadius + bloom) + 1.5;
    } else if (b.style == STYLE_BH) {
        halfSize = bhUnitPx(radiusPx) * BH_EXTENT + 1.5;
    }

    vec2 centrePx = vec2(b.x, b.y) * uScale.y;
    // The quad is clamped to the viewport (plus a margin): a planet far larger than the screen then covers it with a
    // normal-sized quad, and the fragment shader works from gl_FragCoord, so nothing is interpolated across 1e6 pixels.
    vec2 viewHalf = 0.5 * uViewport.xy + 2.0;
    vec2 lo = max(centrePx - halfSize, -viewHalf);
    vec2 hi = min(centrePx + halfSize, viewHalf);
    if (lo.x > hi.x || lo.y > hi.y) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);             // entirely off-screen: culled
    } else {
        gl_Position = uProj * vec4(mix(lo, hi, corner01), 0.0, 1.0);
    }

    vCentre = centrePx;
    vDrawRadius = drawRadius;
    vBody = uint(gl_InstanceID);
    vMarker = radiusPx < drawRadius ? 1.0 : 0.0;
}
