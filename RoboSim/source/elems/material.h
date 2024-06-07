#pragma once
#include <glm/glm.hpp>

struct OBxyz {
    // Ox variables
    glm::vec3 Ox_s{ 0.0f, 0.0f, 0.0f }; // Ox start point
    glm::vec3 Ox_e{ 1000.0f, 0.0f, 0.0f }; // Ox end point

    // Oy variables
    glm::vec3 Oy_s{ 0.0f, 0.0f, 0.0f }; // Oy start point
    glm::vec3 Oy_e{ 0.0f, 1000.0f, 0.0f }; // Oy end point

    // Oz variables
    glm::vec3 Oz_s{ 0.0f, 0.0f, 0.0f }; // Oz start point
    glm::vec3 Oz_e{ 0.0f, 0.0f, 1000.0f }; // Oz end point

};


struct Material {
    glm::vec3 mColor{ 0.7f, 0.8f, 0.8f };
    float mMetallic{ 0.6f };
    float mRoughness{ 0.3f };
    float mAo{ 0.5f };
    glm::vec3 position{ 0.0f, 0.0f, 0.0f };
    glm::vec3 rotation{ 0.0f, 0.0f, 0.0f };
    OBxyz mOxyz;


};

