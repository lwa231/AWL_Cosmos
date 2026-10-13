#version 330 core

layout(location = 0) in vec4 position;
uniform vec2 uHalfExtent; //half the visible world size
uniform vec2 uOffset;
uniform vec2 uStarPos;
uniform float uScale;
const float LZ = 0.4; //light tilt towards camera

out vec2 vLocal;
out vec3 vLightDir;

void main()
{
   vec2 world = position.xy * uScale + uOffset;

   vec2 toStar = uStarPos - uOffset;

   vec2 dir = (length(toStar) > 0.0001) ? normalize(toStar) : vec2(0.0);

   vLightDir = normalize(vec3(dir, LZ));

   vLocal = position.xy;
   gl_Position = vec4(world / uHalfExtent, position.z, 1.0);
}
