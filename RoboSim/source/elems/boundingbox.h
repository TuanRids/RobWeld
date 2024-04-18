#pragma once
#include <glm/glm.hpp>
#include <pch.h>
#include <vector>

namespace nelems
{
    class BoundingBox
    {
    public:
        // Default constructor
        BoundingBox() = default;
        // Constructor to create a bounding box from vertices and indices
        BoundingBox(const std::vector<glm::vec3>& vertices, const std::vector<unsigned int>& indices);

        // Getter methods
        const std::vector<glm::vec3>& get_vertices() const;
        const std::vector<unsigned int>& get_indices() const;

        // Method to check if the bounding box is initialized
        bool is_initialized() const { return !mVertices.empty() && !mIndices.empty(); }


    private:
        // Private members to store vertices and indices
        std::vector<glm::vec3> mVertices;
        std::vector<unsigned int> mIndices;
    };
}
