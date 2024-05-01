#pragma once

#include "pch.h"

#include "render/render_base.h"
#include "vertex_holder.h"
#include "elems/element.h"
#include "render/opengl_buffer_manager.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <chrono>
#include "material.h"
#include <mutex>

namespace nelems {


    /*
    This class is used to create mesh object with default values
    */
    struct oMesh {
        // materials with default values
        Material oMaterial { { 0.0f, 0.0f, 0.0f },
                            0.2f, 0.2f, 1.0f };
        // default object id
        long long ID{ 0 };
        char oname[256] = { 0 };
        bool selected{ false };
        bool hide{false };
        // VAO VBO buffer
        std::vector<VertexHolder> mVertices;
        std::vector<unsigned int> mVertexIndices;
        std::shared_ptr<nrender::VertexIndexBuffer> mRenderBufferMgr;
        //--------------------------------------------------------------------------------
        void init()  { mRenderBufferMgr = std::make_shared<nrender::OpenGL_VertexIndexBuffer>();create_buffers();}

        // create buffers for object to render
        void create_buffers() { mRenderBufferMgr->create_buffers(mVertices, mVertexIndices); }
        // del object buffers
        void delete_buffers() 
        {  
            if (mRenderBufferMgr != nullptr)   
                {mRenderBufferMgr->delete_buffers();}
        }
        //-------------------------------------------------------------------------------- 
        // Transformation Matrix
            void rotate(float angleX, float angleY, float angleZ);
            void move(float offsetX, float offsetY, float offsetZ);


        //--------------------------------------------------------------------------------
        // add vertex to object
        void add_vertex(const VertexHolder& vertex) { mVertices.push_back(vertex); }
        // add vertex index to object
        void add_vertex_index(unsigned int vertex_idx) { mVertexIndices.push_back(vertex_idx); }
        // get vertex indices size of the object
        std::size_t get_vertex_indices_size() const {return mVertexIndices.size();}
        // get vertices size of the object
        std::size_t get_vertices_size() const {return mVertices.size();}
        // get vertex indices
        std::vector<unsigned int> get_vertex_indices() { return mVertexIndices; }

        //--------------------------------------------------------------------------------
        void render() { if (hide) { return; } mRenderBufferMgr->draw(static_cast<int>(mVertexIndices.size()));}
        void render_lines() { if (hide) { return; } mRenderBufferMgr->draw_lines(static_cast<int>(mVertexIndices.size()));}
        // bind object buffers to render
        void bind() { mRenderBufferMgr->bind(); }
        // unbind object buffers
        void unbind() { mRenderBufferMgr->unbind(); }

        //--------------------------------------------------------------------------------

        void changeName( std::string newvalue) {
            strncpy_s(oname, sizeof(oname), newvalue.c_str(), _TRUNCATE);
        }

    };

    /*
    This class is using singleton pattern for creating only 1 instance of mMesh
    We also use observer pattern to notify the changes to the other classes

    */
    class mMesh {
    public:
        // ************** Load & structure mesh **************
        //load mesh from file
        bool load(const std::string& filepath);
        // random number generator based on current time
        long long getCurrentTimeMillis(int size);
        // create mesh from aimesh
        void load_specific_mesh(const aiMesh* mesh, oMesh& outMesh);
        // clear all meshes
        void clear_meshes();
        // update for shader
        void update(nshaders::Shader* shader, bool lightenable);
        void createGridSys();

        // ************** Check and get pointer **************
        // get mesh pointer based on index
        void get_mesh_ptr(int& j, oMesh*& mesh);
        // get mesh pointer based on id
        void get_mesh_ptr(long long ids, oMesh*& mesh);
        // destructor
        virtual ~mMesh() { clear_meshes(); }
        // get size of mesh
        int size() { return mMeshes->size(); }
        // add new mesh to vector
        void pushback(oMesh mesh) { mMeshes->push_back(std::move(mesh));  }
        // create instance
        static mMesh& getInstance() 
        { 
            std::lock_guard<std::mutex> lock(mMutex);
            static mMesh instance;
            return instance; }
        // get address
        std::shared_ptr<std::vector<oMesh>> getMesh() const { return mMeshes; }
        //check selected: 0 Not selected, 1 selected, 2 more than 1 selected
        int check_selected() {
            if (!mMeshes) {
				return 0;
			}
            int count = 0;
            for (int i = 0; i < mMeshes->size(); i++) {
				if (mMeshes->at(i).selected) {
                    count++;
                    if (count > 1) {
						return 2;
					}
				}
			}
            return count;
        }
    private:
        static std::mutex mMutex;
        // add a var to draw 10x10 grid for coordinate system
        std::unique_ptr<oMesh> mCoorSystem;
        std::shared_ptr<std::vector<oMesh>> mMeshes;
        mMesh(){if (!mMeshes) {mMeshes = std::make_shared<std::vector<oMesh>>();}}
    };
}

