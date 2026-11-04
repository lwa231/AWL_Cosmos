#version 450 core
// Coverage around the segment axis (1-px smooth edge); round caps only where vCaps says so (free ends). Mitred ends are
// cut by the quad geometry itself, so neighbouring segments never blend twice at a joint. Colour interpolates along it.
#include "common.glsl"

layout(location = 0) in vec2 vPos;
layout(location = 1) flat in float vLength;
layout(location = 2) flat in float vHalfWidth;
layout(location = 3) flat in vec4 vColour0;
layout(location = 4) flat in vec4 vColour1;
layout(location = 5) flat in vec2 vCaps;

layout(location = 0) out vec4 fragColour;

void main()
{
    float pastStart = vCaps.x > 0.5 ? max(-vPos.x, 0.0) : 0.0;              // distance past a round-capped end
    float pastEnd = vCaps.y > 0.5 ? max(vPos.x - vLength, 0.0) : 0.0;
    float dist = length(vec2(max(pastStart, pastEnd), vPos.y));
    float coverage = clamp(vHalfWidth - dist + 0.5, 0.0, 1.0);
    if (coverage <= 0.0) discard;

    float u = vLength > 0.0 ? clamp(vPos.x / vLength, 0.0, 1.0) : 0.0;
    vec4 c = mix(vColour0, vColour1, u);
    float a = c.a * coverage;
    fragColour = vec4(c.rgb * a, a);                              // premultiplied
}
