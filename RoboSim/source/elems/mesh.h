#pragma once

#include "pch.h"

#include "render/render_base.h"
#include "vertex_holder.h"
#include "elems/element.h"
#include "render/opengl_buffer_manager.h"

#pragma warning(push) 
#pragma warning( disable : 26495) //3rd party library
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#pragma warning(pop)
#include <Eigen/Dense>
#include <chrono>
#include "material.h"
#include <mutex>
#include <memory>

namespace nelems {


    /*
    This class is used to create mesh object with default values
    */
    struct oMesh {
        // materials with default values
        Material oMaterial;
        // default object id
        long long ID{ 0 };
        char oname[256] = { 0 };
        bool selected{ false };
        bool hide{ false };

        // VAO VBO buffer
        std::vector<VertexHolder> mVertices;
        std::vector<unsigned int> mVertexIndices;

        std::shared_ptr<nrender::VertexIndexBuffer> mRenderBufferMgr;

        //--------------------------------------------------------------------------------
        void init() { mRenderBufferMgr = std::make_shared<nrender::OpenGL_VertexIndexBuffer>(); create_buffers(); }

        // create buffers for object to render
        void create_buffers() { mRenderBufferMgr->create_buffers(mVertices, mVertexIndices); }
        // del object buffers
        void delete_buffers(){if (mRenderBufferMgr != nullptr){ mRenderBufferMgr->delete_buffers(); }}

        //-------------------------------------------------------------------------------- 
        // Transformation Matrix
        void rotate(float angleX, float angleY, float angleZ);
        void rotate(float angleX, float angleY, float angleZ, const glm::vec3 pt_center);
        void move(float offsetX, float offsetY, float offsetZ);
        void applyTransformation(const glm::vec3& center, Eigen::Matrix4f transform);

        //--------------------------------------------------------------------------------
        // add vertex to object
        void add_vertex(const VertexHolder& vertex) { mVertices.push_back(vertex); }
        // add vertex index to object
        void add_vertex_index(unsigned int vertex_idx) { mVertexIndices.push_back(vertex_idx); }
        // get vertex indices size of the object
        std::size_t get_vertex_indices_size() const { return mVertexIndices.size(); }
        // get vertices size of the object
        std::size_t get_vertices_size() const { return mVertices.size(); }
        // get vertex indices
        std::vector<unsigned int> get_vertex_indices() { return mVertexIndices; }
        void calculate_normals();

        //--------------------------------------------------------------------------------
        void render() { if (hide) { return; } mRenderBufferMgr->draw(static_cast<int>(mVertexIndices.size())); }
        void render_lines() { if (hide) { return; } mRenderBufferMgr->draw_lines(static_cast<int>(mVertexIndices.size())); }
        // bind object buffers to render
        void bind() { mRenderBufferMgr->bind(); }
        // unbind object buffers
        void unbind() { mRenderBufferMgr->unbind(); }

        //--------------------------------------------------------------------------------

        void changeName(std::string newvalue) {
            strncpy_s(oname, sizeof(oname), newvalue.c_str(), _TRUNCATE);
        }
    };

    
    class mMesh {
    public:
        // ************** Load & structure mesh **************
        //load mesh from file
        bool load(const std::string& filepath, bool robot);
        long long getCurrentTimeMillis(int size);
        void load_specific_mesh(const aiMesh* mesh, oMesh& outMesh);
        void clear_meshes();
        void update(nshaders::Shader* shader, int lightmode);
        void createGridSys(float size, float step);

        // ************** Check and get pointer **************
        static mMesh& getInstance()
        {
            std::lock_guard<std::mutex> lock(mMutex);
            static mMesh instance;
            return instance;
        }
        virtual ~mMesh() { clear_meshes(); }
        std::shared_ptr<std::vector<std::shared_ptr<oMesh>>> getMesh() const { return mMeshes; }

        int check_selected() {
            if (!mMeshes || mMeshes->empty()) { return 0; }
            return std::count_if(mMeshes->begin(), mMeshes->end(), [](const std::shared_ptr<oMesh> &mesh)
                {return mesh->selected; });
        }
        void delete_byname(const std::string& delmesh);
        void add_mesh(std::shared_ptr<oMesh> addnewmesh);
        void set_axis_length(const int& length) { axis_length = length; }

    private:
        static std::mutex mMutex;
        std::unique_ptr<oMesh> mCoorSystem;
        std::shared_ptr<std::vector<std::shared_ptr<oMesh>>> mMeshes;
        int axis_length{ 1000 };

        mMesh() { if (!mMeshes) { mMeshes = std::make_shared<std::vector<std::shared_ptr<oMesh>>>(); } }
        void set_OBxyz(float length, oMesh& mesh, oMesh& OBox, oMesh& OBoy, oMesh& OBoz);
    };
}
