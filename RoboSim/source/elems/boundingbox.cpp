#include <pch.h>
#include "boundingbox.h"


namespace nelems
{
    BoundingBox::BoundingBox(const std::vector<glm::vec3>& vertices, const std::vector<unsigned int>& indices)
    {
        if (vertices.empty())
            return;

        // Calculate the minimum and maximum coordinates of the bounding box
        float minX = vertices[0].x, minY = vertices[0].y, minZ = vertices[0].z;
        float maxX = vertices[0].x, maxY = vertices[0].y, maxZ = vertices[0].z;

        for (const auto& vertex : vertices)
        {
            if (vertex.x < minX)
                minX = vertex.x;
            if (vertex.x > maxX)
                maxX = vertex.x;

            if (vertex.y < minY)
                minY = vertex.y;
            if (vertex.y > maxY)
                maxY = vertex.y;

            if (vertex.z < minZ)
                minZ = vertex.z;
            if (vertex.z > maxZ)
                maxZ = vertex.z;
        }

        // Create the vertices of the bounding box
        mVertices.push_back(glm::vec3(minX, minY, minZ));
        mVertices.push_back(glm::vec3(maxX, minY, minZ));
        mVertices.push_back(glm::vec3(maxX, maxY, minZ));
        mVertices.push_back(glm::vec3(minX, maxY, minZ));
        mVertices.push_back(glm::vec3(minX, minY, maxZ));
        mVertices.push_back(glm::vec3(maxX, minY, maxZ));
        mVertices.push_back(glm::vec3(maxX, maxY, maxZ));
        mVertices.push_back(glm::vec3(minX, maxY, maxZ));

        // Define the indices of the bounding box edges
        mIndices = {
            0, 1, 1, 2, 2, 3, 3, 0, // Front face
            4, 5, 5, 6, 6, 7, 7, 4, // Back face
            0, 4, 1, 5, 2, 6, 3, 7  // Connect front and back faces
        };
    }

    const std::vector<glm::vec3>& BoundingBox::get_vertices() const
    {
        return mVertices;
    }

    const std::vector<unsigned int>& BoundingBox::get_indices() const
    {
        return mIndices;
    }

    

}
