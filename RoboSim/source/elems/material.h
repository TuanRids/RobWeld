#pragma once
#include <glm/glm.hpp>
typedef struct Material {
    glm::vec3 mColor;
    float metallic;
    float roughness;
    float ao;
    glm::vec3 position{ 0.0f,0.0f,0.0f };
    glm::vec3 rotation{ 0.0f,1.0f,2.0f };
};