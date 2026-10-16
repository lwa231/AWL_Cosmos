#version 330 core

// MAX_LIGHTS is #defined by the app at load time (see InjectDefine in Shader.cpp)
const float LZ = 0.4;          // light tilt towards camera

in vec2 vLocal;
uniform vec3  uColor;
uniform float uLit;            // 1 = shaded by lights, 0 = drawn flat (stars)
uniform float uAmbient;        // faint light on the night side
uniform vec2  uOffset;         // this body's center (also used by the vertex shader)
uniform int   uLightCount;
uniform vec2  uLightPos[MAX_LIGHTS];
uniform vec3  uLightColor[MAX_LIGHTS];

layout(location = 0) out vec4 color;

void main()
{
   float r2 = dot(vLocal, vLocal);
   vec3 n = vec3(vLocal, sqrt(max(0.0, 1.0 - r2)));

   vec3 light = vec3(0.0);
   for (int i = 0; i < uLightCount; ++i)
   {
      vec2 toLight = uLightPos[i] - uOffset;
      vec2 dir = (length(toLight) > 0.0001) ? normalize(toLight) : vec2(0.0);
      vec3 L = normalize(vec3(dir, LZ));
      light += uLightColor[i] * max(dot(n, L), 0.0);
   }

   vec3 lit = min(uColor * (uAmbient + light), vec3(1.0));
   color = vec4(mix(uColor, lit, uLit), 1.0);
}
