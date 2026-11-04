#version 450 core
// Sphere impostors: one instanced quad per body, the disc is carved in the fragment shader.
#include "common.glsl"
#include "bodies.glsl"

layout(location = 0) out vec2  vLocal;      // quad coordinates scaled so the disc has radius 1
layout(location = 1) flat out float vDrawRadius;   // on-screen disc radius in pixels (>= half the minimum marker size)
layout(location = 2) flat out uint  vBody;
layout(location = 3) flat out float vMarker;       // 1 when drawn as a minimum-size marker

void main()
{
    Body b = bodies[gl_InstanceID];
    vec2 corner = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1)) * 2.0 - 1.0;   // triangle strip order

    float radiusPx = b.radius * uScale.y;
    float drawRadius = max(radiusPx, 0.5 * uScale.w);
    float halfSize = drawRadius + 1.5;                      // margin for the anti-aliased edge

    vec2 centrePx = vec2(b.x, b.y) * uScale.y;
    gl_Position = uProj * vec4(centrePx + corner * halfSize, 0.0, 1.0);

    vLocal = corner * (halfSize / drawRadius);
    vDrawRadius = drawRadius;
    vBody = uint(gl_InstanceID);
    vMarker = radiusPx < drawRadius ? 1.0 : 0.0;
}
