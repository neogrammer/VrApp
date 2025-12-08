#version 300 es
precision mediump float;

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec2 inUV;  // same layout as mesh, but unused

layout(std140) uniform CameraConstants
{
    mat4 viewProj;
    mat4 modelViewProj;
    mat4 model;
    vec4 color;
    vec4 pad1;
    vec4 pad2;
    vec4 pad3;
};

out vec4 vColor;

void main()
{
    gl_Position = modelViewProj * inPosition;
    vColor      = color;
}