#version 450 core
// Capsule coverage around the segment (distance to the segment, 1-px smooth edge); colour interpolated along it.
#include "common.glsl"

layout(location = 0) in vec2 vPos;
layout(location = 1) flat in float vLength;
layout(location = 2) flat in float vHalfWidth;
layout(location = 3) flat in vec4 vColour0;
layout(location = 4) flat in vec4 vColour1;

layout(location = 0) out vec4 fragColour;

void main()
{
    float outside = max(max(-vPos.x, vPos.x - vLength), 0.0);    // distance past either end along the axis
    float dist = length(vec2(outside, vPos.y));
    float coverage = clamp(vHalfWidth - dist + 0.5, 0.0, 1.0);
    if (coverage <= 0.0) discard;

    float u = vLength > 0.0 ? clamp(vPos.x / vLength, 0.0, 1.0) : 0.0;
    vec4 c = mix(vColour0, vColour1, u);
    float a = c.a * coverage;
    fragColour = vec4(c.rgb * a, a);                              // premultiplied
}
