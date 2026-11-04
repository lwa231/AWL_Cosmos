#version 450 core
// Streamed line batch: every segment of every polyline is one instanced quad (4 vertices, triangle strip) extended by
// half a width at both ends and shaded as a capsule, so consecutive segments join without gaps. glLineWidth > 1 is not
// reliable in a core profile, hence quads. The segment is first clipped to the viewport (in pixels) so a polyline
// that is mostly far off-screen (a large orbit at high zoom) keeps full floating-point precision.
#include "common.glsl"

struct Segment {
    vec4 p;    // x0, y0, x1, y1 (camera-relative metres)
    vec4 c0;   // rgba at the start (straight alpha)
    vec4 c1;   // rgba at the end
    vec4 w;    // x: width in pixels
};
layout(std430, binding = 3) readonly buffer SegmentBuffer { Segment segments[]; };

layout(location = 0) out vec2 vPos;                // x: pixels along the segment from the clipped start, y: across
layout(location = 1) flat out float vLength;       // clipped length in pixels
layout(location = 2) flat out float vHalfWidth;    // pixels
layout(location = 3) flat out vec4 vColour0;       // colour at the clipped start
layout(location = 4) flat out vec4 vColour1;       // colour at the clipped end

void main()
{
    Segment s = segments[gl_InstanceID];
    vec2 a = s.p.xy * uScale.y;
    vec2 b = s.p.zw * uScale.y;
    float halfWidth = 0.5 * max(s.w.x, 0.0);
    float reach = halfWidth + 1.0;                 // + 1 px for the anti-aliased edge

    // Liang-Barsky clip against the viewport grown by `reach`
    vec2 limit = 0.5 * uViewport.xy + reach;
    vec2 d = b - a;
    float t0 = 0.0, t1 = 1.0;
    bool visible = true;
    for (int axis = 0; axis < 2; ++axis) {
        float p0 = -d[axis], q0 = a[axis] + limit[axis];     // left/bottom boundary
        float p1 =  d[axis], q1 = limit[axis] - a[axis];     // right/top boundary
        if (p0 == 0.0) { if (q0 < 0.0) visible = false; }
        else { float r = q0 / p0; if (p0 < 0.0) t0 = max(t0, r); else t1 = min(t1, r); }
        if (p1 == 0.0) { if (q1 < 0.0) visible = false; }
        else { float r = q1 / p1; if (p1 < 0.0) t0 = max(t0, r); else t1 = min(t1, r); }
    }
    if (!visible || t0 > t1) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);    // outside the clip volume: culled
        vPos = vec2(0.0); vLength = 0.0; vHalfWidth = 0.0; vColour0 = vec4(0.0); vColour1 = vec4(0.0);
        return;
    }

    vec2 a2 = a + d * t0;
    vec2 b2 = a + d * t1;
    float len = length(b2 - a2);
    vec2 axisDir = len > 1e-6 ? (b2 - a2) / len : vec2(1.0, 0.0);
    vec2 perp = vec2(-axisDir.y, axisDir.x);

    float along = ((gl_VertexID & 1) == 0) ? -reach : len + reach;
    float across = ((gl_VertexID >> 1) == 0) ? -reach : reach;
    vec2 px = a2 + axisDir * along + perp * across;
    gl_Position = uProj * vec4(px, 0.0, 1.0);

    vPos = vec2(along, across);
    vLength = len;
    vHalfWidth = halfWidth;
    vColour0 = mix(s.c0, s.c1, t0);
    vColour1 = mix(s.c0, s.c1, t1);
}
