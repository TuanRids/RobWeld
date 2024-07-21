#include <pch.h>
#include "TranRBMatrix.h"

using namespace Eigen;
using namespace nelems;

void TranRBMatrix::applyTransformation(oMesh& mesh, const glm::vec3& center, float angleX, float angleY, float angleZ) {
    // create rotation matrix
    Matrix4f transform = Matrix4f::Identity();

    // X-axis rotation
    if (angleX != 0.0f) {
        float rad = glm::radians(angleX);
        Matrix4f rotX;
        rotX << 1, 0, 0, 0,
            0, cos(rad), -sin(rad), 0,
            0, sin(rad), cos(rad), 0,
            0, 0, 0, 1;
        transform *= rotX;
    }

    // Y-axis rotation
    if (angleY != 0.0f) {
        float rad = glm::radians(angleY);
        Matrix4f rotY;
        rotY << cos(rad), 0, sin(rad), 0,
            0, 1, 0, 0,
            -sin(rad), 0, cos(rad), 0,
            0, 0, 0, 1;
        transform *= rotY;
    }

    // Z-axis rotation
    if (angleZ != 0.0f) {
        float rad = glm::radians(angleZ);
        Matrix4f rotZ;
        rotZ << cos(rad), -sin(rad), 0, 0,
            sin(rad), cos(rad), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        transform *= rotZ;
    }

    Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);

    for (auto& vertex : mesh.mVertices) {
        Vector4f pos(vertex.mPos.x - center.x, vertex.mPos.y - center.y, vertex.mPos.z - center.z, 1.0f);
        Vector4f newPos = transform * pos;
        vertex.mPos = glm::vec3(newPos.x() + center.x, newPos.y() + center.y, newPos.z() + center.z);
    }

    // Update oMaterial position
    Vector4f centerPos(mesh.oMaterial.position.x - center.x, mesh.oMaterial.position.y - center.y, mesh.oMaterial.position.z - center.z, 1.0f);
    Vector4f newCenterPos = transform * centerPos;
    mesh.oMaterial.position = glm::vec3(newCenterPos.x() + center.x, newCenterPos.y() + center.y, newCenterPos.z() + center.z);

    // Update oMaterial rotation
    Quaternionf q(rotationMatrix);
    q.normalize();
    Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    mesh.oMaterial.rotation.x += glm::degrees(euler.x());
    mesh.oMaterial.rotation.y += glm::degrees(euler.y());
    mesh.oMaterial.rotation.z += glm::degrees(euler.z());
}
