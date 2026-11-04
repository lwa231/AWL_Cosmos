#version 450 core
// Full-screen triangle for the gravity-well ring pass (plan 9.4 step 2). No vertex data: the corners come from gl_VertexID.
void main()
{
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));   // (0,0) (2,0) (0,2)
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
