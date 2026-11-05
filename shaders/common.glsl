// Shared by every pass (pulled in with #include "common.glsl"; the C++ loader expands it).
//
// Convention: the frame uniform block carries an orthographic projection from CENTRE-ORIGIN PIXELS (x right, y up,
// camera at 0,0) to clip space. Object positions arrive as camera-relative METRES; shaders multiply by pixels-per-metre
// (uScale.y) first and only then apply uProj, so the big numbers of a 1e11 m scene never touch the matrix.
//
// Output colours are PREMULTIPLIED alpha and the blend function is (ONE, ONE_MINUS_SRC_ALPHA) for every pass.
// A fragment that writes alpha = 0 therefore adds its colour to the framebuffer (the additive flame).
layout(std140, binding = 0) uniform FrameData {
    mat4  uProj;       // centre-origin pixels -> clip space
    vec4  uViewport;   // x: width, y: height, z: 1/width, w: 1/height (pixels)
    vec4  uScale;      // x: metres per pixel, y: pixels per metre, z: time (s, wrapped), w: minimum body diameter (px)
    uvec4 uCounts;     // x: bodies, y: lights, z: shuttles
    vec4  uShuttle;    // x: hull length (m), y: minimum hull length (px)
    vec4  uVisual;     // x: body quality (0 Low, 1 Medium, 2 High), y: skybox quality, z: skybox seed (0..1)
};
