#include "pch.h"
#include "mesh.h"



#include "render/opengl_buffer_manager.h"

namespace nelems
{
    bool mMesh::load(const std::string& filepath)
    {
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
                load_specific_mesh(mesh, newMesh);
                mMeshes->push_back(newMesh);
            }

            return true;
        }
        return false;
    }
    long long mMesh::getCurrentTimeMillis(int size)
    {
        auto now = std::chrono::system_clock::now();
        auto t_c = std::chrono::system_clock::to_time_t(now);

        std::tm t;
        localtime_s(&t, &t_c);

        long long millis = (t.tm_mon + 1) * 100000000000LL +
            t.tm_mday * 1000000000LL +
            t.tm_hour * 10000000LL +
            t.tm_min * 100000LL +
            t.tm_sec * 1000LL;


        bool uniqueIDFound = false;

        // Generate a random number from the C++11 random environment
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<long long> dis(0, size + 2); // Random range 
        do {
            // Get the current time
            auto now = std::chrono::system_clock::now();
            // Combine the current time with a random number
            millis = millis + dis(gen);
            // Check if the new ID already exists in the list of existing IDs
            bool found = false;
            for (const auto& mesh : *mMeshes) {
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
    void mMesh::load_specific_mesh(const aiMesh* mesh, oMesh& outMesh)
    {
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
    void mMesh::clear_meshes()
    {
        if (mMeshes == nullptr) return;
        for (auto& mesh : *mMeshes) {
            mesh.delete_buffers();
        }
        mMeshes->clear();
    }
    void mMesh::update(nshaders::Shader* shader)
    {
        for (auto& mesh : *mMeshes) {
            shader->set_material(mesh.oMaterial, "materialData");
            mesh.render();
        }
    }
    void mMesh::render()
    {
        for (auto& mesh : *mMeshes)
        {
            // Render each mesh
            mesh.render();
        }
    }
    void mMesh::get_mesh_ids(std::vector<long long>& ids)
    {
        for (auto& mesh : *mMeshes)
        {
            ids.push_back(mesh.ID);
        }
    }
    void mMesh::get_mesh_ptr(int& j, oMesh*& mesh)
    {
        for (int i = 0; i < mMeshes->size(); ++i) {
            if (i == j)
            {
                mesh = &(*mMeshes)[i]; //dereference the pointer to get the object
                return;
            }

        }
    }
    void mMesh::get_mesh_ptr(long long ids, oMesh*& mesh)
    {
        for (auto& m : *mMeshes) {
            if (m.ID == ids) {
                mesh = &m;
                return;
            }
        }
    }
}
