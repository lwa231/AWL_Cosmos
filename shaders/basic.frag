#version 330 core

in vec2 vLocal;
uniform vec3  uColor;
in vec3 vLightDir;    // unit vector from this body toward the star
uniform float uLit;         // 1 = lit planet/moon, 0 = glowing (the star itself)

layout(location = 0) out vec4 color;

void main()
{
   float r2 = dot(vLocal, vLocal);
   vec3 n = vec3(vLocal, sqrt(max(0.0, 1.0 - r2)));
   float diffuse = max(dot(n, vLightDir), 0.0);
   vec3 lit = uColor * (0.04 + diffuse); //0.04 = faint ambient light

   color = vec4(mix(uColor, lit, uLit), 1.0);
}
