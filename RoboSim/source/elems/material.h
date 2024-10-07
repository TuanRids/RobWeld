#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

#include <Eigen/Dense>
struct OBxyz {
    float size{ 300.0f };
    glm::vec3 xs{ 0.0f, 0.0f, 0.0f }; // Ox start point
    glm::vec3 xe{ size, 0.0f, 0.0f }; // Ox end point

    glm::vec3 ys{ 0.0f, 0.0f, 0.0f }; // Oy start point
    glm::vec3 ye{ 0.0f, size, 0.0f }; // Oy end point

    glm::vec3 zs{ 0.0f, 0.0f, 0.0f }; // Oz start point
    glm::vec3 ze{ 0.0f, 0.0f, size }; // Oz end point

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

struct BoundingBox {
    glm::vec3 min, max;
    Eigen::Vector3f corners[8];

    void build_corners() {
        corners[0] = Eigen::Vector3f(min.x, min.y, min.z);
        corners[1] = Eigen::Vector3f(min.x, max.y, min.z);
        corners[2] = Eigen::Vector3f(max.x, max.y, min.z);
        corners[3] = Eigen::Vector3f(max.x, min.y, min.z);
        corners[4] = Eigen::Vector3f(min.x, min.y, max.z);
        corners[5] = Eigen::Vector3f(min.x, max.y, max.z);
        corners[6] = Eigen::Vector3f(max.x, max.y, max.z);
        corners[7] = Eigen::Vector3f(max.x, min.y, max.z);
    }
    bool intersect(const glm::vec3& ray_origin, const glm::vec3& ray_dir) const {
        float tmin = (min.x - ray_origin.x) / ray_dir.x;
        float tmax = (max.x - ray_origin.x) / ray_dir.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        float tymin = (min.y - ray_origin.y) / ray_dir.y;
        float tymax = (max.y - ray_origin.y) / ray_dir.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        float tzmin = (min.z - ray_origin.z) / ray_dir.z;
        float tzmax = (max.z - ray_origin.z) / ray_dir.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        return true;
    }
    /*bool intersect(const glm::vec3& ray_origin, const glm::vec3& ray_dir) const {
        Eigen::Vector3f rayOrig(ray_origin.x, ray_origin.y, ray_origin.z);
        Eigen::Vector3f rayDir(ray_dir.x, ray_dir.y, ray_dir.z);

        for (int i = 0; i < 8; ++i) {
            for (int j = i + 1; j < 8; ++j) {
                Eigen::Vector3f edge = corners[j] - corners[i];
                Eigen::Vector3f edge_dir = edge.normalized();

                Eigen::Vector3f oc = rayOrig - corners[i];
                float t = oc.dot(edge_dir.cross(rayDir)) / rayDir.dot(edge_dir.cross(rayDir));

                if (t >= 0) {
                    return true; 
                }
            }
        }
        return false; 
    }*/

};


struct Material {
    glm::vec3 mColor{ 0.7f, 0.8f, 0.8f };
    float mMetallic{ 0.6f };
    float mRoughness{ 0.3f };
    float mAo{ 0.5f };
    glm::vec3 position{ 0.0f, 0.0f, 0.0f };
    glm::vec3 rotation{ 0.0f, 0.0f, 0.0f };
    OBxyz mOxyz;
    BoundingBox BBox;

};

