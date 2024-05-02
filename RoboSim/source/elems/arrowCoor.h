#pragma once
#include <glm/glm.hpp>
#include <pch.h>
#include <vector>
#include "shader/shader_util.h"
namespace nelems
{
    class ArrowCoor
    {
    public:
        void drawArrow(nshaders::Shader* shader, const glm::vec3& base, int alen) const {
            glm::vec3 tip = base + glm::vec3(0.0f, alen, 0.0f); 
            glm::vec3 direction = tip - base;
            float length = glm::length(direction);
            float radius = length * 0.05f;
            glm::mat4 model = glm::translate(glm::mat4(1.0f), base);
            glm::vec3 axis = glm::normalize(direction);
            float angle = glm::acos(glm::dot(axis, glm::vec3(0.0f, 1.0f, 0.0f)));
            glm::vec3 rotationAxis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), axis);
            model = glm::rotate(model, angle, rotationAxis);
            model = glm::scale(model, glm::vec3(radius, length, radius));
            shader->set_mat4(model, "model");
        }

    };
}
