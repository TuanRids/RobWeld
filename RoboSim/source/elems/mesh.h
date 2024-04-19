#pragma once

#include "pch.h"

#include "render/render_base.h"
#include "vertex_holder.h"
#include "elems/element.h"
#include "render/opengl_buffer_manager.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <random>
#include <chrono>
namespace nelems {

    struct oMesh {
        glm::vec3 mColor = { 1.0f, 0.0f, 0.0f };
        float mRoughness = 0.2f;
        float mMetallic = 0.1f;
        long long ID;
        std::vector<VertexHolder> mVertices;
        std::vector<unsigned int> mVertexIndices;
        std::shared_ptr<nrender::VertexIndexBuffer> mRenderBufferMgr;

        void create_buffers() { mRenderBufferMgr->create_buffers(mVertices, mVertexIndices); }

        void delete_buffers() { mRenderBufferMgr->delete_buffers(); }

        std::size_t get_vertex_indices_size() const {return mVertexIndices.size();}

        std::size_t get_vertices_size() const {return mVertices.size();}

        void render() { mRenderBufferMgr->draw(static_cast<int>(mVertexIndices.size())); }

        void bind() { mRenderBufferMgr->bind(); }

        void add_vertex(const VertexHolder& vertex) { mVertices.push_back(vertex); }

        void add_vertex_index(unsigned int vertex_idx) { mVertexIndices.push_back(vertex_idx); }

        std::vector<unsigned int> get_vertex_indices() { return mVertexIndices; }
        
        void unbind() { mRenderBufferMgr->unbind(); }

        void init()
        { 
            mRenderBufferMgr = std::make_unique<nrender::OpenGL_VertexIndexBuffer>();
            create_buffers();
        }

    };

    class mMesh {
    public:
        mMesh() = default;

        virtual ~mMesh() { clear_meshes(); }

        bool load(const std::string& filepath) {
            const uint32_t cMeshImportFlags =
                aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_SortByPType |
                aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_OptimizeMeshes |
                aiProcess_ValidateDataStructure;

            Assimp::Importer Importer;

            const aiScene* pScene =
                Importer.ReadFile(filepath.c_str(), cMeshImportFlags);

            if (pScene && pScene->HasMeshes()) {
                // Load all meshes
                for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                    auto* mesh = pScene->mMeshes[i];
                    oMesh newMesh;
                    newMesh.ID = getCurrentTimeMillis(pScene->mNumMeshes);
                    load_mesh(mesh, newMesh);
                    mMeshes.push_back(newMesh);
                }
                
                return true;
            }
            return false;
        }
        long long getCurrentTimeMillis(int size) {
            auto now = std::chrono::system_clock::now();
            auto t_c = std::chrono::system_clock::to_time_t(now);

            std::tm t;
            localtime_s(&t, &t_c);

            long long millis = (t.tm_mon + 1)   * 100000000000LL +
                                t.tm_mday       * 1000000000LL +
                                t.tm_hour       * 10000000LL +
                                t.tm_min        * 100000LL +
                                t.tm_sec        * 1000LL;


            bool uniqueIDFound = false;

            // Generate a random number from the C++11 random environment
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<long long> dis(0, size+2); // Random range 
            do {
                // Get the current time
                auto now = std::chrono::system_clock::now();
                // Combine the current time with a random number
                millis = millis + dis(gen);
                // Check if the new ID already exists in the list of existing IDs
                bool found = false;
                for (const auto& mesh : mMeshes) {
                    if (mesh.ID == millis) {
                        found = true;
                        break;
                    }
                }

                // If the new ID is not a duplicate, exit the loop
                if (!found) {
                    uniqueIDFound = true;
                }
            } while (!uniqueIDFound);

            return millis;
        }

        void load_mesh(const aiMesh* mesh, oMesh& outMesh) {
            for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
                VertexHolder vh;
                vh.mPos = { mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z };
                vh.mNormal = { mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z };
                outMesh.mVertices.push_back(vh);
            }

            for (size_t i = 0; i < mesh->mNumFaces; i++) {
                aiFace face = mesh->mFaces[i];
                for (size_t j = 0; j < face.mNumIndices; j++)
                    outMesh.mVertexIndices.push_back(face.mIndices[j]);
            }

            outMesh.init();
        }

        void clear_meshes() {
            for (auto& mesh : mMeshes) {
                mesh.delete_buffers();
            }
            mMeshes.clear();
        }

        void update(nshaders::Shader* shader) {
            std::vector<float> a;
            for (auto& mesh : mMeshes) {
                a.push_back(mesh.mRoughness);
                shader->set_vec3(mesh.mColor, "albedo");
                shader->set_f1(mesh.mRoughness, "roughness");
                shader->set_f1(mesh.mMetallic, "metallic");
                shader->set_f1(1.0f, "ao");
            }
        }

        void render() {
            for (auto& mesh : mMeshes) 
            {
                // Render each mesh
                mesh.render();
            }
        }
        
        void get_mesh_ids(std::vector<long long> &ids) 
        {
            for (auto& mesh : mMeshes) 
            {
				ids.push_back(mesh.ID);
			}
        }

        void get_mesh_ptr(long long ids, oMesh*& mesh) {
            for (auto& m : mMeshes) {
                if (m.ID == ids) {
                    mesh = &m;
                    return;

                }
            }
        }

    private:
        std::vector<oMesh> mMeshes;
    };

}
