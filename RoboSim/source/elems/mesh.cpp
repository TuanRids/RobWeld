#include "pch.h"
#include "mesh.h"


#include <filesystem>
#include "render/opengl_buffer_manager.h"
#include <thread>


/// <summary>
/// oMesh
/// </summary>
namespace nelems {
    void oMesh::rotate(float angleX, float angleY, float angleZ)
    {
        // Calculate the rotation matrix
        glm::mat4 rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleX), glm::vec3(1.0f, 0.0f, 0.0f));
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleY), glm::vec3(0.0f, 1.0f, 0.0f));
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleZ), glm::vec3(0.0f, 0.0f, 1.0f));

        // Translate the vertices to the origin of the coordinate system
        glm::mat4 translationToOriginMatrix = glm::translate(glm::mat4(1.0f), -oMaterial.position);

        // Translate the vertices back to their original positions after rotation
        glm::mat4 translationBackMatrix = glm::translate(glm::mat4(1.0f), oMaterial.position);

        // Apply the transformation to each vertex
        glm::mat4 modelMatrix = translationBackMatrix * rotationMatrix * translationToOriginMatrix;

        for (auto& vertex : mVertices) {
            glm::vec4 newPosition = modelMatrix * glm::vec4(vertex.mPos, 1.0f);
            vertex.mPos = glm::vec3(newPosition);
        }
    }

    void oMesh::move(float offsetX, float offsetY, float offsetZ)
    {
        // Update the center point
        oMaterial.position.x += offsetX;
        oMaterial.position.y += offsetY;
        oMaterial.position.z += offsetZ;

        // Update the positions of all vertices
        for (auto& vertex : mVertices) {
            vertex.mPos.x += offsetX;
            vertex.mPos.y += offsetY;
            vertex.mPos.z += offsetZ;
        }
    }
}


/// <summary>
/// mMesh
/// </summary>
namespace nelems
{
    std::mutex nelems::mMesh::mMutex;
    //TODO: Load files too slow
    bool mMesh::load(const std::string& filepath)
    {
        const uint32_t cMeshImportFlags =
            aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_SortByPType |
            aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_OptimizeMeshes |
            aiProcess_ValidateDataStructure ;

        Assimp::Importer Importer;

        const aiScene* pScene =
            Importer.ReadFile(filepath.c_str(), cMeshImportFlags);
        std::string filename = std::filesystem::path(filepath).filename().stem().string() + " ";
        if (pScene && pScene->HasMeshes()) {
            // Load all meshes
            for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                auto* mesh = pScene->mMeshes[i];
                oMesh newMesh;
                newMesh.ID = getCurrentTimeMillis(pScene->mNumMeshes);
                newMesh.changeName(filename + std::to_string(newMesh.ID % 1000));
                load_specific_mesh(mesh, newMesh);
                mMeshes->push_back(newMesh);
            }
            return true;
        }
        return false;
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
        // Calculate the center of all vertices
        glm::vec3 center = glm::vec3(0.0f);
        for (const auto& vertex : outMesh.mVertices) {
            center += vertex.mPos;
        }
        center /= static_cast<float>(outMesh.mVertices.size());
        outMesh.oMaterial.position = center;
        outMesh.oMaterial.mColor = glm::vec3(0.0f, 0.5f, 1.0f);
        outMesh.init();
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

        do {
            // Get the current time
            auto now = std::chrono::system_clock::now();
            // Combine the current time with a random number
            millis++;
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
    void mMesh::clear_meshes()
    {
        if (mMeshes == nullptr) return;
        for (auto& mesh : *mMeshes) {
            std::cout<<mesh.mRenderBufferMgr<<std::endl;
            mesh.delete_buffers();
        }
        mMeshes->clear();
        if (mCoorSystem == nullptr) return;
        for (auto& mesh : *mCoorSystem) {
            std::cout << mesh.mRenderBufferMgr << std::endl;
            mesh.delete_buffers();
        }
         mCoorSystem->clear();
    }
    void mMesh::update(nshaders::Shader* shader, bool lightsEnabled)
    {
        if (lightsEnabled == 0){shader->set_i1(0, "lightsEnabled");}
		else {shader->set_i1(1, "lightsEnabled");}

        for (auto& mesh : *mMeshes) {
            shader->set_material(mesh.oMaterial, "materialData");
            mesh.render();
        }
        shader->set_material(mCoorSystem->at(0).oMaterial, "materialData");
        mCoorSystem->at(0).render_lines();
        
    }
    void mMesh::createGridSys(int gridNo, int step)
    {
        if (!mCoorSystem)
        {
            int mini_no = 2;
            mCoorSystem = std::make_shared<std::vector<oMesh>>(2);
            mCoorSystem->at(0).changeName("Coordinate System");

            mCoorSystem->at(0).oMaterial.mColor = glm::vec3(0.0f, 0.0f, 0.0f);
            mCoorSystem->at(0).ID = getCurrentTimeMillis(0);

            // Create vertices and indices for the main grid
            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    // Set the position of the vertex
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->at(0).add_vertex(vertex);
                    // Add horizontal index if not at the last column
                    if (x < gridNo / 2.0f - 1) {
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f));
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f + 1) * gridNo + (y + gridNo / 2.0f));
                    }
                    // Add vertical index if not at the last row
                    if (y < gridNo / 2.0f - 1) {
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f));
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f + 1));
                    }
                }
            }
            mCoorSystem->at(0).init();

        }
        else {
            mCoorSystem->at(0).delete_buffers();
            mCoorSystem->at(0).mVertices.clear();
            mCoorSystem->at(0).mVertexIndices.clear();
            // Create vertices and indices for the main grid
            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    // Set the position of the vertex
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->at(0).add_vertex(vertex);
                    // Add horizontal index if not at the last column
                    if (x < gridNo / 2.0f - 1) {
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f));
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f + 1) * gridNo + (y + gridNo / 2.0f));
                    }
                    // Add vertical index if not at the last row
                    if (y < gridNo / 2.0f - 1) {
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f));
                        mCoorSystem->at(0).add_vertex_index((x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f + 1));
                    }
                }
            }
            mCoorSystem->at(0).init();
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
