#include "pch.h"
#include "mesh.h"

#include "map"
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
        // Update the rotation of the object
        oMaterial.rotation.x += angleX;
        oMaterial.rotation.y += angleY;
        oMaterial.rotation.z += angleZ;
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

    //Loadsync
    bool mMesh::load_sync(const std::string& filepath) {
        static std::ifstream file;
        static std::streampos last_pos = 0;

        // Open the file
        if (!file.is_open()) {
            file.open(filepath, std::ios::binary);
            if (!file.is_open()) {
                // Failed to open the file
                return false;
            }
        }

        // Move to the last position
        file.seekg(last_pos);

        // Read the file
        std::stringstream buffer;
        buffer << file.rdbuf();

        // Remember the last position
        last_pos = file.tellg();

        // Process the data from the buffer
        

        // Close the file if it's reached the end
        if (file.eof()) {
            file.close();
            // Reset the last position for the next file
            last_pos = 0;
        }

        return true;
    }


    //TODO: Load files too slow
    bool mMesh::load(const std::string& filepath, bool robot)
    {
        // Predefined colors for materials
        static glm::vec3 color[9] = {
            glm::vec3(0.14, 0.14, 0.14), glm::vec3(0.13, 0.15, 0.17), glm::vec3(0.1, 0.4, 0.7),
            glm::vec3(0.0, 0.5, 0.0), glm::vec3(1.0, 0.8, 0.2), glm::vec3(0.0, 0.6, 0.4),
            glm::vec3(0.2, 0.1, 0.1), glm::vec3(0.04, 0.04, 0.4), glm::vec3(0.4, 0.1, 0.7)
        };

        // Import flags for Assimp
        const uint32_t cMeshImportFlags =
            aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_OptimizeMeshes |
            aiProcess_ValidateDataStructure;

        // Create an Assimp importer instance
        Assimp::Importer Importer;

        // Load the scene
        const aiScene* pScene = Importer.ReadFile(filepath.c_str(), cMeshImportFlags);

        // Extract filename without extension
        std::string filename = std::filesystem::path(filepath).filename().stem().string() + " ";

        // Check if the scene and meshes exist
        if (pScene && pScene->HasMeshes()) {
            // Map to store center points of meshes with 'ct' names
            static std::map<std::string, glm::vec3> ctMap;

            // Processing for robot meshes
            if (robot)
            {
                // Iterate through meshes to find 'ct' named meshes and store their center points
                for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                    auto* mesh = pScene->mMeshes[i];
                    std::string name = mesh->mName.C_Str();
                    if (name.find("ct") != std::string::npos) {
                        // Extract the index from mesh name and store the center point
                        ctMap[name] = glm::vec3(mesh->mVertices[0].x, mesh->mVertices[0].y, mesh->mVertices[0].z);
                    }
                }
            }

            // Counter for color index
            static int color_idx = 0;

            // Load all meshes
            for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                auto* mesh = pScene->mMeshes[i];
                oMesh newMesh;
                newMesh.ID = getCurrentTimeMillis(pScene->mNumMeshes);
                load_specific_mesh(mesh, newMesh);
                if (robot)
                {
                    std::string name = mesh->mName.C_Str();
                    if (name.find("ct") != std::string::npos) { continue; } // Skip 'ct' named meshes
                    // Assign mesh name, color, and position
                    newMesh.changeName(name);
                    newMesh.oMaterial.mColor = color[color_idx++];
                    // Extract index from mesh name and append to 'ct' for position lookup
                    name = "ct" + name.substr(name.find_last_of(" ") + 1);
                    // Retrieve center point from ctMap and assign to material position
                    newMesh.oMaterial.position = ctMap[name];
                }
                else {
                    // Assign default name for non-robot meshes
                    newMesh.changeName(filename + std::to_string(newMesh.ID % 1000));
                }
                mMeshes->push_back(newMesh);
            }
            return true; // Loading successful
        }
        return false; // Loading failed
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
        //outMesh.oMaterial.mColor = glm::vec3(0.0f, 0.5f, 1.0f);
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
            mesh.delete_buffers();
        }
        mMeshes->clear();
        if (mCoorSystem == nullptr) return;
        mCoorSystem->delete_buffers();
        mCoorSystem.reset();

    }
    void mMesh::update(nshaders::Shader* shader, int lightmode)
    {
        shader->set_i1(lightmode, "LightModes");
        for (auto& mesh : *mMeshes) {
            shader->set_material(mesh.oMaterial, "materialData");
            mesh.render();
            mesh.unbind();
        }
        for (auto& mesh : *mMeshes) {
            if (!mesh.selected) { continue; }
            oMesh OBxyz, OBox, OBoy, OBoz;
            set_OBxyz(axis_length, mesh, OBox, OBoy, OBoz);
            shader->set_material(OBox.oMaterial, "materialData");
            OBox.render_lines();    shader->set_material(OBoy.oMaterial, "materialData");
            OBoy.render_lines();	shader->set_material(OBoz.oMaterial, "materialData");
            OBoz.render_lines();
        }
        shader->set_i1(2, "LightModes");
        shader->set_material(mCoorSystem->oMaterial, "materialData");
        mCoorSystem->render_lines();
    }
    void mMesh::createGridSys(float gridNo, float step)
    {
        float vlue;
        if (!mCoorSystem)
        {
            int mini_no = 2;
            //mCoorSystem = std::make_shared<std::vector<oMesh>>(2);
            mCoorSystem = std::make_unique<oMesh>();
            mCoorSystem->changeName("Coordinate System");

            mCoorSystem->oMaterial.mColor = glm::vec3(0.8f, 0.7f, 0.75f);
            mCoorSystem->ID = getCurrentTimeMillis(0);
            
            // Create vertices and indices for the main grid
            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    // Set the position of the vertex
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->add_vertex(vertex);
                    // Add horizontal index if not at the last column
                    if (x < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f + 1.0f)* gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
                    // Add vertical index if not at the last row
                    if (y < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f)* gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f + 1.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
                }
            }
            mCoorSystem->init();

        }
        else {
            mCoorSystem->delete_buffers();
            mCoorSystem->mVertices.clear();
            mCoorSystem->mVertexIndices.clear();
            // Create vertices and indices for the main grid
            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    // Set the position of the vertex
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->add_vertex(vertex);
                    // Add horizontal index if not at the last column
                    if (x < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f + 1) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
                    // Add vertical index if not at the last row
                    if (y < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f + 1);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
                }
            }
            mCoorSystem->init();
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
    void mMesh::delete_selected()
    {
        for (auto& mesh : *mMeshes)
        {
            if (mesh.selected)
            {
                mesh.delete_buffers();
            }
        }
        mMeshes->erase(std::remove_if(mMeshes->begin(), mMeshes->end(),
            [](const oMesh& mesh) { return mesh.selected; }), mMeshes->end());
    }


    void mMesh::set_OBxyz(float length,oMesh& mesh, oMesh& OBox, oMesh& OBoy, oMesh& OBoz)
    {

        glm::mat4 rotationMatrix;

        glm::vec3 position = mesh.oMaterial.position;
        glm::vec3 rotation = mesh.oMaterial.rotation;

        
        rotationMatrix = glm::rotate(glm::mat4(1.0f), rotation.x, glm::vec3(1, 0, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.y, glm::vec3(0, 1, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.z, glm::vec3(0, 0, 1));
        glm::vec3 ox_vector = glm::vec3(rotationMatrix * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        glm::vec3 ox_end_point = position + glm::normalize(ox_vector) * length;

        rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, rotation.x, glm::vec3(1, 0, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.y, glm::vec3(0, 1, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.z, glm::vec3(0, 0, 1));
        glm::vec3 oy_vector = glm::vec3(rotationMatrix * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
        glm::vec3 oy_end_point = position + glm::normalize(oy_vector) * length;

        rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, rotation.x, glm::vec3(1, 0, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.y, glm::vec3(0, 1, 0));
        rotationMatrix = glm::rotate(rotationMatrix, rotation.z, glm::vec3(0, 0, 1));
        glm::vec3 oz_vector = glm::vec3(rotationMatrix * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
        glm::vec3 oz_end_point = position + glm::normalize(oz_vector) * length;

        // Ox mesh (Red color)
        oMesh oxMesh;
        oxMesh.mVertices.clear();
        oxMesh.mVertexIndices.clear();
        oxMesh.add_vertex(VertexHolder(position, glm::vec3(1.0f, 0.0f, 0.0f)));
        oxMesh.add_vertex(VertexHolder(ox_end_point, glm::vec3(1.0f, 0.0f, 0.0f)));
        oxMesh.add_vertex_index(0);
        oxMesh.add_vertex_index(1);
        oxMesh.init();
        OBox = oxMesh;
        OBox.oMaterial.mColor = glm::vec3(1.0f, 0.0f, 0.0f);
        // Oy mesh (Cyan color)
        oMesh oyMesh;
        oyMesh.mVertices.clear();
        oyMesh.mVertexIndices.clear();
        oyMesh.add_vertex(VertexHolder(position, glm::vec3(0.0f, 1.0f, 1.0f)));
        oyMesh.add_vertex(VertexHolder(oy_end_point, glm::vec3(0.0f, 1.0f, 1.0f)));
        oyMesh.add_vertex_index(0);
        oyMesh.add_vertex_index(1);
        oyMesh.init();
        OBoy = oyMesh;
        OBoy.oMaterial.mColor = glm::vec3(0.0f, 1.0f, 1.0f);
        // Oz mesh (Purple color)
        oMesh ozMesh;
        ozMesh.mVertices.clear();
        ozMesh.mVertexIndices.clear();
        ozMesh.add_vertex(VertexHolder(position, glm::vec3(0.5f, 0.0f, 0.5f)));
        ozMesh.add_vertex(VertexHolder(oz_end_point, glm::vec3(0.5f, 0.0f, 0.5f)));
        ozMesh.add_vertex_index(0);
        ozMesh.add_vertex_index(1);
        ozMesh.init();
        OBoz = ozMesh;
        OBoz.oMaterial.mColor = glm::vec3(0.5f, 0.0f, 0.5f);
    }

}
