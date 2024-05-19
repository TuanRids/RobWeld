#pragma once

#include <vector>
#include <glm/glm.hpp>

struct Bone {
    glm::vec3 position; 
    glm::vec3 rotation; 
    std::vector<Bone*> children; 
};

class RiggingSystem {
private:
    std::vector<Bone> bones; 
public:
    RiggingSystem() {}

    void addBone(const glm::vec3& position, const glm::vec3& rotation) {
        Bone newBone;
        newBone.position = position;
        newBone.rotation = rotation;
        bones.push_back(newBone);
    }

    void setBoneHierarchy(int parentIndex, int childIndex) {
        if (parentIndex >= 0 && parentIndex < bones.size() &&
            childIndex >= 0 && childIndex < bones.size()) {
            bones[parentIndex].children.push_back(&bones[childIndex]);
        }
    }

    void controlBone(int boneIndex, const glm::vec3& newPosition, const glm::vec3& newRotation) {
        if (boneIndex >= 0 && boneIndex < bones.size()) {
            bones[boneIndex].position = newPosition;
            bones[boneIndex].rotation = newRotation;
        }
    }

    void applyRiggingToMeshes() {
    }
};
