#version 330 core

layout(location = 0) in vec4 position;
uniform vec2 uHalfExtent; //half the visible world size
uniform vec2 uOffset;
uniform float uScale;

out vec2 vLocal;

void main()
{
   vec2 world = position.xy * uScale + uOffset;

   vLocal = position.xy;
   gl_Position = vec4(world / uHalfExtent, position.z, 1.0);
}
