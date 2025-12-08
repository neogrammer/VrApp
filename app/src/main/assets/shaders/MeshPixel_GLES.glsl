#version 310 es

layout(location = 0) in mediump vec2 v_TexCoord;
layout(location = 1) in highp vec3 v_Color;  // still there, we just ignore it

layout(location = 0) out highp vec4 o_Color;

// Keep binding 2 uniform block so layout still matches your pipeline,
// but we don't use it.
layout(std140, binding = 2) uniform Data {
    highp vec4 colors[6];
} d_Data;

void main() {
    // Visualize UVs:
    //  - Red  = U
    //  - Green = V
    //  - Blue = 0
    o_Color = vec4(v_TexCoord, 0.0, 1.0);
}