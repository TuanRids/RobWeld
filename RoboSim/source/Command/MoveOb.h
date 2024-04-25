// MoveCommand.h
#pragma once
#include "Command/dcommand.h"
#include "elems/mesh.h"
#include <glm/glm.hpp>
#include "Command/cmdManage.h"
namespace ncommand
{
    class MoveOb : public Command {
    private:
        nelems::oMesh* mesh;
        glm::vec3 originalPosition;
        glm::vec3 MoveVec3;

    public:
        MoveOb(nelems::oMesh* obj, float x, float y, float z) : mesh(obj), originalPosition(obj->oMaterial.position) {
            MoveVec3 = glm::vec3(x, y, z);
        }
        void execute() override {
            // Calculate displacement vector
            glm::vec3 displacementVector = MoveVec3;
            // Update vertex positions
            for (auto& vertex : mesh->mVertices) {
                vertex.mPos += displacementVector;
            }
            // Update buffers
            mesh->create_buffers();
        }

        void undo() override {
            // Restore original position
            glm::vec3 displacementVector = MoveVec3;
            // Update vertex positions
            for (auto& vertex : mesh->mVertices) {
                vertex.mPos -= displacementVector;
            }
            // Update buffers
            mesh->create_buffers();
        }

        void redo() override {
            // Execute the move again
        }
    };


    class RotateOb : public Command
    {
    private:
        nelems::oMesh* mesh;
        glm::vec3 rotVec3;
    public:
        RotateOb(nelems::oMesh* obj, float x) : mesh(obj) {
            rotVec3 = glm::vec3(x);
        }
        void execute() override {
        }
        void undo() override {}
        void redo() override {}
    };
}
