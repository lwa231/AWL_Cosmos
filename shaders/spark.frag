#version 450 core
// Soft round speck, written ADDITIVELY: premultiplied colour with alpha 0 under the (ONE, ONE_MINUS_SRC_ALPHA) blend adds to
// the framebuffer (see common.glsl).
layout(location = 0) in vec2 vCorner;
layout(location = 1) in vec4 vColour;
layout(location = 0) out vec4 fragColour;

void main()
{
    float d = length(vCorner);
    float cover = clamp((1.0 - d) * 1.6, 0.0, 1.0);
    if (cover <= 0.0 || vColour.a <= 0.0) discard;
    fragColour = vec4(vColour.rgb * (vColour.a * cover), 0.0);
}
