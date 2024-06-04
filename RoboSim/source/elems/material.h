#pragma once
#include <glm/glm.hpp>

struct Line {
    glm::vec3 start;
    glm::vec3 end;
};

struct Material {
    glm::vec3 mColor{ 0.7f, 0.8f, 0.8f };
    float mMetallic{ 0.6f };
    float mRoughness{ 0.3f };
    float mAo{ 0.5f };
    glm::vec3 position{ 0.0f,0.0f,0.0f };
    glm::vec3 rotation{ 0.0f,0.0f,0.0f };

};
