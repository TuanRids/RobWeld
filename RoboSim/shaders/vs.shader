#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec4 color;

out vec3 WorldPos;
out vec3 Normal;
out vec4 Color;

void main() {
    Color = color;
    // An undetermined problem cause that the y axis is switched with the z axis.
    //  ************ Switch y axis and z axis together ************
    // Transform the vertex position to world space
    WorldPos = vec3(model * vec4(aPosition.x, aPosition.z, aPosition.y, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;

    // Apply the model-view-projection transformation to the vertex position
    gl_Position = projection * view * model * vec4(aPosition.x, aPosition.z, aPosition.y, 1.0f);
}
