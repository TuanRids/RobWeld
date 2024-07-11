#version 430 core

layout(local_size_x = 256) in;

struct Vertex {
    vec3 position;
    vec3 normal;
};

layout(std430, binding = 0) buffer VertexBuffer {
    Vertex vertices[];
};

uniform mat4 transform;
uniform vec3 center;

void main() {
    uint index = gl_GlobalInvocationID.x;
    vec4 newPos = transform * vec4(vertices[index].position - center, 1.0);
    vertices[index].position = vec3(newPos) + center;
}
