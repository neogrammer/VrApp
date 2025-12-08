#version 300 es
precision highp float;

layout(location = 0) in vec4 inPosition;

void main()
{
    // Scale the model down a bit so it fits nicely into clip space.
    vec3 p = inPosition.xyz * 0.3;
    gl_Position = vec4(p, 1.0);
}