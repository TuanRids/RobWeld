#pragma once
#include <glm/glm.hpp>

struct Line {
    glm::vec3 start;
    glm::vec3 end;
};

struct Material {
    glm::vec3 mColor{ 0.0f, 0.0f, 0.0f };
    float mMetallic{ 0.2f };
    float mRoughness{ 0.2f };
    float mAo{ 0.5f };
    float mTransparency{ 0.5f };
    glm::vec3 position{ 0.0f,0.0f,0.0f };
    glm::vec3 rotation{ 0.0f,0.0f,0.0f };

};
