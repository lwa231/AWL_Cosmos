#version 450 core
// Streamed line batch: every segment of every polyline is one instanced quad (4 vertices, triangle strip). glLineWidth > 1
// is not reliable in a core profile, hence quads. The segment is first clipped to the viewport (in pixels) so a polyline
// that is mostly far off-screen (a large orbit at high zoom) keeps full floating-point precision.
//
// Joins: consecutive segments of a strip are MITRED. Both quads end on the same bisector line with bit-identical corner
// positions, so a pixel at a joint is covered exactly once (no gap, no overlap) and a translucent line has no brighter
// "bead" at its vertices. Only free ends (strip start/end, loose segments) and joints too sharp to mitre get a round
// cap, drawn as a capsule in the fragment shader.
#include "common.glsl"

struct Segment {
    vec4 p;    // x0, y0, x1, y1 (camera-relative metres)
    vec4 c0;   // rgba at the start (straight alpha)
    vec4 c1;   // rgba at the end
    float width;   // pixels
    int prev;      // index of the segment joined to the start, or -1
    int next;      // index of the segment joined to the end, or -1
    int pad;
};
layout(std430, binding = 3) readonly buffer SegmentBuffer { Segment segments[]; };

layout(location = 0) out vec2 vPos;                // x: pixels along the segment from the clipped start, y: across
layout(location = 1) flat out float vLength;       // clipped length in pixels
layout(location = 2) flat out float vHalfWidth;    // pixels
layout(location = 3) flat out vec4 vColour0;       // colour at the clipped start
layout(location = 4) flat out vec4 vColour1;       // colour at the clipped end
layout(location = 5) flat out vec2 vCaps;          // 1 where the start / end is a round cap (0: mitred or clipped)

const float MITER_MIN_COS = 0.5;                   // turns sharper than 120 degrees get a round cap instead of a mitre
const float MIN_LENGTH_PX = 1e-3;

// Direction and length (pixels) of a segment, computed identically wherever it is needed.
bool segmentDir(int index, out vec2 dir, out float len)
{
    Segment q = segments[index];
    vec2 d = (q.p.zw - q.p.xy) * uScale.y;
    len = length(d);
    dir = len > MIN_LENGTH_PX ? d / len : vec2(1.0, 0.0);
    return len > MIN_LENGTH_PX;
}

// Mitre between segment A (arriving) and B (leaving). m is the unit bisector of their left normals; a corner on the left /
// right side is  joint + m * (+-reach / k).  Fails for near-reversals, sharp turns and segments too short for the mitre.
bool mitre(vec2 dirA, float lenA, vec2 dirB, float lenB, float reach, out vec2 m, out float k)
{
    vec2 nA = vec2(-dirA.y, dirA.x), nB = vec2(-dirB.y, dirB.x);
    vec2 sum = nA + nB;
    float sl = length(sum);
    m = vec2(0.0); k = 1.0;
    if (sl < 1e-4) return false;
    m = sum / sl;
    k = dot(m, nA);
    if (k < MITER_MIN_COS) return false;
    float shift = reach * abs(dot(m, dirA)) / k;       // how far the corner slides along the segment
    return shift <= 0.5 * min(lenA, lenB);
}

void main()
{
    Segment s = segments[gl_InstanceID];
    vec2 a = s.p.xy * uScale.y;
    vec2 b = s.p.zw * uScale.y;
    float halfWidth = 0.5 * max(s.width, 0.0);
    float reach = halfWidth + 1.0;                 // + 1 px for the anti-aliased edge

    vec2 dir; float L;
    segmentDir(gl_InstanceID, dir, L);
    vec2 d = b - a;

    // Liang-Barsky clip against the viewport grown by `reach`
    vec2 limit = 0.5 * uViewport.xy + reach;
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
        vPos = vec2(0.0); vLength = 0.0; vHalfWidth = 0.0; vColour0 = vec4(0.0); vColour1 = vec4(0.0); vCaps = vec2(0.0);
        return;
    }

    const bool startClipped = t0 > 0.0;
    const bool endClipped = t1 < 1.0;
    vec2 a2 = startClipped ? a + d * t0 : a;       // exact endpoints when unclipped, so shared corners match bit for bit
    vec2 b2 = endClipped ? a + d * t1 : b;
    float len = (t1 - t0) * L;
    vec2 perp = vec2(-dir.y, dir.x);

    // Joins at the two ends (only where the end is not clipped away)
    vec2 mStart = vec2(0.0), mEnd = vec2(0.0);
    float kStart = 1.0, kEnd = 1.0;
    bool joinStart = false, joinEnd = false;
    if (!startClipped && s.prev >= 0) {
        vec2 pd; float pl;
        if (segmentDir(s.prev, pd, pl) && L > MIN_LENGTH_PX) joinStart = mitre(pd, pl, dir, L, reach, mStart, kStart);
    }
    if (!endClipped && s.next >= 0) {
        vec2 nd; float nl;
        if (segmentDir(s.next, nd, nl) && L > MIN_LENGTH_PX) joinEnd = mitre(dir, L, nd, nl, reach, mEnd, kEnd);
    }

    bool isStart = (gl_VertexID & 1) == 0;
    float side = ((gl_VertexID >> 1) == 0) ? -1.0 : 1.0;
    vec2 px;
    if (isStart) {
        if (joinStart) px = a + mStart * (side * reach / kStart);
        else           px = a2 + dir * (startClipped ? 0.0 : -reach) + perp * (side * reach);   // free end: room for a round cap
    } else {
        if (joinEnd) px = b + mEnd * (side * reach / kEnd);
        else         px = b2 + dir * (endClipped ? 0.0 : reach) + perp * (side * reach);
    }
    gl_Position = uProj * vec4(px, 0.0, 1.0);

    vec2 rel = px - a2;
    vPos = vec2(dot(rel, dir), dot(rel, perp));
    vLength = len;
    vHalfWidth = halfWidth;
    vColour0 = mix(s.c0, s.c1, t0);
    vColour1 = mix(s.c0, s.c1, t1);
    vCaps = vec2((!startClipped && !joinStart) ? 1.0 : 0.0, (!endClipped && !joinEnd) ? 1.0 : 0.0);
}
