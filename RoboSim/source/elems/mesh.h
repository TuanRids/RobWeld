#pragma once

#include "pch.h"

#include "render/render_base.h"
#include "vertex_holder.h"
#include "elems/element.h"
#include "boundingbox.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
namespace nelems
{
  class Mesh : public Element
  {
    
  public:

    Mesh() = default;

    virtual ~Mesh();
    
    bool load(const std::string& filepath,int nuMesh);
    void writeloadmesh(const aiMesh* mesh);
    void add_vertex(const VertexHolder& vertex) { mVertices.push_back(vertex);  }

    void add_vertex_index(unsigned int vertex_idx) { mVertexIndices.push_back(vertex_idx); }

    std::vector<unsigned int> get_vertex_indices() { return mVertexIndices; }

    std::size_t get_vertex_indices_size() const
    {
        return mVertexIndices.size();
    }
    std::size_t get_vertices_size() const
    {
        return mVertices.size();
    }

    void update(nshaders::Shader* shader) override
    {
      // pbr color
      shader->set_vec3(mColor,      "albedo");
      shader->set_f1(mRoughness,    "roughness");
      shader->set_f1(mMetallic,     "metallic");
      shader->set_f1(1.0f,          "ao");

    }
    
    glm::vec3 mColor = { 1.0f, 0.0f, 0.0f };
    float mRoughness = 0.2f;
    float mMetallic = 0.1f;

    void init();

    void create_buffers();

    void delete_buffers();

    void render();

    void bind();

    void unbind();

    // Method to initialize the bounding box
    void init_bounding_box();

    // Method to get the bounding box
    BoundingBox get_bounding_box() const { return mBoundingBox; }
    std::vector<glm::vec3> get_bbox_vertices(std::vector<glm::vec3> bbox) { return mBoundingBox.get_vertices(); }
    std::vector<unsigned int> get_bbox_indices() const { return mBoundingBox.get_indices(); }

  private:
    // Buffers manager
    std::unique_ptr<nrender::VertexIndexBuffer> mRenderBufferMgr;
    
    // Vertices and indices
    std::vector<VertexHolder> mVertices;
    std::vector<unsigned int> mVertexIndices;

    // Bounding box
    BoundingBox mBoundingBox;
  };
}

