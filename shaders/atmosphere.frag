#version 450 core
// alpha = peak * (rho(h) / rho0)^power with rho = rho0 exp(-h / H), so alpha = peak * exp(-power * h / H) (plan 9.5), faded
// to zero over the outer third of the shell so the cutoff has no visible edge. Premultiplied output.
#include "common.glsl"

struct Atmosphere {
    float x, y;
    float radius;
    float top;
    float scaleHeight;
    float r, g, b;
};
layout(std430, binding = 5) readonly buffer AtmosphereBuffer { Atmosphere atmospheres[]; };

layout(location = 0) uniform vec2 uAtm;   // x: peak alpha at the surface, y: power

layout(location = 0) flat in vec2  vCentre;
layout(location = 1) flat in float vRadiusPx;
layout(location = 2) flat in float vTopPx;
layout(location = 3) flat in uint  vAtm;

layout(location = 0) out vec4 fragColour;

void main()
{
    Atmosphere a = atmospheres[vAtm];
    vec2 rel = gl_FragCoord.xy - 0.5 * uViewport.xy - vCentre;
    float hPx = max(length(rel) - vRadiusPx, 0.0);                // height above the surface; the body itself covers r < R
    if (hPx >= vTopPx) discard;

    float alpha = uAtm.x * exp(-uAtm.y * hPx * uScale.x / a.scaleHeight);
    alpha *= 1.0 - smoothstep(0.65 * vTopPx, vTopPx, hPx);
    fragColour = vec4(vec3(a.r, a.g, a.b) * alpha, alpha);
}
