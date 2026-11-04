#version 450 core
// Hill: solid cyan. SOI: dashed green. Roche: dotted orange. All 1.5 px wide at any zoom. Premultiplied output.
#include "common.glsl"

layout(location = 0) flat in vec2  vCentre;
layout(location = 1) flat in float vRadiusPx;
layout(location = 2) flat in uint  vKind;
layout(location = 3) flat in float vDashes;

layout(location = 0) out vec4 fragColour;

const float HALF_WIDTH = 0.75;

void main()
{
    vec2 rel = gl_FragCoord.xy - 0.5 * uViewport.xy - vCentre;
    float ring = abs(length(rel) - vRadiusPx);                    // pixels from the rim
    float cover = clamp(HALF_WIDTH + 0.5 - ring, 0.0, 1.0);
    if (cover <= 0.0) discard;

    vec3 colour;
    float alpha;
    if (vKind == 0u) {                       // Hill: solid
        colour = vec3(0.35, 0.85, 1.00);
        alpha = 0.80;
    } else {
        float duty = vKind == 1u ? 0.60 : 0.30;
        float period = 6.2831853 * vRadiusPx / vDashes;                // pixels per dash cycle
        float v = fract(atan(rel.y, rel.x) * (vDashes / 6.2831853));   // 0..1 along one cycle
        float toEnd = clamp((duty - v) * period + 0.5, 0.0, 1.0);      // pixels left to the dash end
        float fromStart = clamp(v * period + 0.5, 0.0, 1.0);           // pixels since the dash start
        cover *= min(toEnd, fromStart);
        if (cover <= 0.0) discard;
        if (vKind == 1u) { colour = vec3(0.45, 1.00, 0.50); alpha = 0.80; }   // SOI: dashed
        else             { colour = vec3(1.00, 0.55, 0.25); alpha = 0.90; }   // Roche: dotted
    }
    float a = alpha * cover;
    fragColour = vec4(colour * a, a);
}
