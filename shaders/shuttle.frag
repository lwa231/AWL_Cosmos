#version 450 core
// Hull: three sectors (the part of the triangle nearest each vertex). Flame: blue at the nozzle, orange in the body,
// red at the tip, additive (alpha 0 with the premultiplied blend function adds the colour).
#include "common.glsl"

layout(location = 0) in vec3 vBary;
layout(location = 1) in vec2 vFlame;
layout(location = 2) flat in int  vKind;
layout(location = 3) flat in vec4 vColourA;
layout(location = 4) flat in vec3 vColourB;
layout(location = 5) flat in vec3 vColourC;
layout(location = 6) flat in float vFlicker;

layout(location = 0) out vec4 fragColour;

void main()
{
    if (vKind == 1) {
        vec3 colour = vColourA.rgb;
        if (vBary.y >= vBary.x && vBary.y >= vBary.z) colour = vColourB;
        else if (vBary.z >= vBary.x && vBary.z >= vBary.y) colour = vColourC;
        fragColour = vec4(colour, 1.0);
        return;
    }

    float throttle = vColourA.w;
    float t = clamp(vFlame.y, 0.0, 1.0);
    const vec3 BLUE   = vec3(0.35, 0.60, 1.00);
    const vec3 ORANGE = vec3(1.00, 0.55, 0.10);
    const vec3 RED    = vec3(0.85, 0.08, 0.02);
    vec3 colour = (t < 0.25) ? mix(BLUE, ORANGE, smoothstep(0.0, 0.25, t))
                             : mix(ORANGE, RED, smoothstep(0.25, 1.0, t));

    float halfWidth = mix(1.0, 0.12, t);                          // teardrop: narrows towards the tip
    float lateral = clamp(1.0 - abs(vFlame.x) / halfWidth, 0.0, 1.0);
    float brightness = (0.5 + 1.0 * throttle) * pow(1.0 - t, 1.1) * sqrt(lateral) * vFlicker;
    fragColour = vec4(colour * brightness, 0.0);                  // alpha 0: additive
}
