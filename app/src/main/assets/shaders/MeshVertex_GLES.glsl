#version 310 es

layout(std140, binding = 0) uniform CameraConstants {
    mat4 viewProj;
    mat4 modelViewProj;
    mat4 model;
    vec4 colour;
    vec4 pad1;
    vec4 pad2;
    vec4 pad3;
};

// Keep Normals UBO layout even if we don't use it now, to match your descriptor set.
layout(std140, binding = 1) uniform Normals {
    vec4 normals[6];
};

// location 0: position (vec4)
// location 1: texcoord (vec2)
layout(location = 0) in highp vec4 a_Positions;   // x,y,z,w
layout(location = 1) in mediump vec2 a_TexCoord;  // u,v

layout(location = 0) out mediump vec2 v_TexCoord;
layout(location = 1) out highp vec3 v_Color;

void main() {
    gl_Position = modelViewProj * a_Positions;
    v_TexCoord = a_TexCoord;
    v_Color = colour.rgb;
}

