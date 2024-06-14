#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

struct OBxyz {
    glm::vec3 xs{ 0.0f, 0.0f, 0.0f }; // Ox start point
    glm::vec3 xe{ 1000.0f, 0.0f, 0.0f }; // Ox end point

    glm::vec3 ys{ 0.0f, 0.0f, 0.0f }; // Oy start point
    glm::vec3 ye{ 0.0f, 1000.0f, 0.0f }; // Oy end point

    glm::vec3 zs{ 0.0f, 0.0f, 0.0f }; // Oz start point
    glm::vec3 ze{ 0.0f, 0.0f, 1000.0f }; // Oz end point

    void update(const glm::vec3& position, const glm::vec3& rotation) {
        // Create the rotation matrix from the rotation vector (Euler angles)
        glm::quat quaternion = glm::quat(glm::vec3(glm::radians(rotation.x), glm::radians(rotation.y), glm::radians(rotation.z)));
        glm::mat4 rotationMatrix = glm::toMat4(quaternion);
        // Update StartPoint and Endpoints based on the new position
        xs = position; xe = position + glm::vec3(rotationMatrix * glm::vec4(1000.0f, 0.0f, 0.0f, 1.0f));
        ys = position; ye = position + glm::vec3(rotationMatrix * glm::vec4(0.0f, 1000.0f, 0.0f, 1.0f));
        zs = position; ze = position + glm::vec3(rotationMatrix * glm::vec4(0.0f, 0.0f, 1000.0f, 1.0f));
        
        //  If rotation.x != 0: Rotate ye and ze by rotation.x
        if (rotation.x != 0.0f)
        {
            float radX = glm::radians(rotation.x);
            ye = position + glm::rotate(glm::vec3(0.0f, 1000.0f, 0.0f), radX, glm::vec3(1.0f, 0.0f, 0.0f));
			ze = position + glm::rotate(glm::vec3(0.0f, 0.0f, 1000.0f), radX, glm::vec3(1.0f, 0.0f, 0.0f));
        }
        //  If rotation.y != 0: Rotate xe and ze by rotation.y
        if (rotation.y != 0.0f)
        {
            float radY = glm::radians(rotation.y);
            xe = position + glm::rotate(glm::vec3(1000.0f, 0.0f, 0.0f), radY, glm::vec3(0.0f, 1.0f, 0.0f));
            ze = position + glm::rotate(glm::vec3(0.0f, 0.0f, 1000.0f), radY, glm::vec3(0.0f, 1.0f, 0.0f));
        }
		//  If rotation.z != 0: Rotate xe and ye by rotation.z
        if (rotation.z != 0.0f)
        {
            float radZ = glm::radians(rotation.z);
			xe +=  glm::rotate(glm::vec3(1000.0f, 0.0f, 0.0f), radZ, glm::vec3(0.0f, 0.0f, 1.0f));
			ye +=  glm::rotate(glm::vec3(0.0f, 1000.0f, 0.0f), radZ, glm::vec3(0.0f, 0.0f, 1.0f));
        }

    }
};

struct Material {
    glm::vec3 mColor{ 0.7f, 0.8f, 0.8f };
    float mMetallic{ 0.6f };
    float mRoughness{ 0.3f };
    float mAo{ 0.5f };
    glm::vec3 position{ 0.0f, 0.0f, 0.0f };
    glm::vec3 rotation{ 0.0f, 0.0f, 0.0f };
    OBxyz mOxyz;

    void updateOxyz() {
        mOxyz.update(position, rotation);
    }


};

