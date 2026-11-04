#version 450 core
// Sphere-impostor shading (ported from the original basic.frag): the lit hemisphere gets light from every light,
// plus ambient; stars are flat. Black holes are a dark disc with a thin bright ring.
#include "common.glsl"
#include "bodies.glsl"

layout(location = 0) in vec2 vLocal;
layout(location = 1) flat in float vDrawRadius;
layout(location = 2) flat in uint  vBody;
layout(location = 3) flat in float vMarker;

layout(location = 0) out vec4 fragColour;

const float LZ = 0.4;              // light tilt towards the camera
const float RING_PX = 1.5;         // black-hole ring thickness

void main()
{
    Body b = bodies[vBody];
    vec3 base = vec3(b.r, b.g, b.b);

    float d = length(vLocal);
    float insidePx = (1.0 - d) * vDrawRadius;            // distance inside the rim in pixels
    float coverage = clamp(insidePx + 0.5, 0.0, 1.0);    // smooth 1-px edge
    if (coverage <= 0.0) discard;

    vec3 colour;
    if ((b.flags & BODY_BLACK_HOLE) != 0u) {
        float ring = 1.0 - clamp(insidePx - RING_PX + 0.5, 0.0, 1.0);   // 1 on the outer RING_PX pixels
        colour = base * ring;                                           // dark inside, bright ring
    } else if ((b.flags & BODY_FLAT) != 0u || vMarker > 0.5) {
        colour = base;                                   // stars, unlit bodies and minimum-size markers: flat
    } else {
        vec3 n = vec3(vLocal, sqrt(max(0.0, 1.0 - d * d)));
        vec3 light = vec3(0.0);
        for (uint i = 0u; i < uCounts.y; ++i) {
            Light l = lights[i];
            vec2 toLight = vec2(l.x - b.x, l.y - b.y);
            float m = max(abs(toLight.x), abs(toLight.y));   // scale first so squaring 1e11 m cannot overflow
            vec2 dir = (m > 0.0) ? normalize(toLight / m) : vec2(0.0);
            vec3 L = normalize(vec3(dir, LZ));
            light += vec3(l.r, l.g, l.b) * l.intensity * max(dot(n, L), 0.0);
        }
        colour = min(base * (b.ambient + light), vec3(1.0));
    }
    fragColour = vec4(colour * coverage, coverage);       // premultiplied
}
