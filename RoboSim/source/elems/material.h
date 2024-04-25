#pragma once
#include <glm/glm.hpp>
typedef struct Material {
    glm::vec3 mColor;
    float metallic;
    float roughness;
    float ao;
    glm::vec3 position{ 0.0f,0.0f,0.0f };
};