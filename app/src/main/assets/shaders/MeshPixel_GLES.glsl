#version 310 es
precision mediump float;

layout(std140, binding = 0) uniform CameraConstants {
    mat4 viewProj;
    mat4 modelViewProj;
    mat4 model;
    vec4 colour;
    vec4 pad1;
    vec4 pad2;
    vec4 pad3;
};

layout(std140, binding = 1) uniform Normals {
    vec4 normals[6];
};

layout(location = 0) in mediump vec2 v_TexCoord;
layout(location = 1) in highp   vec3 v_Color;

layout(location = 0) out highp  vec4 o_Color;

layout(binding = 2) uniform sampler2D u_DrumTex;

void main() {
    vec4 tex = texture(u_DrumTex, v_TexCoord);
    o_Color = tex * vec4(v_Color, 1.0);
}