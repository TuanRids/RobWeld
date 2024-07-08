#include "pch.h"
#include "mesh.h"

#include "map"
#include <filesystem>
#include "render/opengl_buffer_manager.h"
#include <unordered_set>
#include <functional>



#include <omp.h>
#include <vector>



namespace nelems {
    void oMesh::rotate(float angleX, float angleY, float angleZ) {
        // Calculate the rotation matrix
        glm::vec3 axisX = glm::normalize(oMaterial.mOxyz.xe - oMaterial.mOxyz.xs);
        glm::vec3 axisY = glm::normalize(oMaterial.mOxyz.ye - oMaterial.mOxyz.ys);
        glm::vec3 axisZ = glm::normalize(oMaterial.mOxyz.ze - oMaterial.mOxyz.zs);

        glm::mat4 rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleX), axisX);
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleY), axisY);
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleZ), axisZ);

        // Apply the transformation to each vertex
        for (auto& vertex : mVertices) {
            glm::vec4 newPosition = rotationMatrix * glm::vec4(vertex.mPos - oMaterial.position, 1.0f);
            vertex.mPos = glm::vec3(newPosition) + oMaterial.position;
        }

        // Update the rotation of the object
        oMaterial.rotation.x += angleX;
        oMaterial.rotation.y += angleY;
        oMaterial.rotation.z += angleZ;

        // Rotate Oxyz
        glm::vec4 newOx_s = rotationMatrix * glm::vec4(oMaterial.mOxyz.xs - oMaterial.position, 1.0f);
        oMaterial.mOxyz.xs = glm::vec3(newOx_s) + oMaterial.position;
        glm::vec4 newOx_e = rotationMatrix * glm::vec4(oMaterial.mOxyz.xe - oMaterial.position, 1.0f);
        oMaterial.mOxyz.xe = glm::vec3(newOx_e) + oMaterial.position;
        glm::vec4 newOy_s = rotationMatrix * glm::vec4(oMaterial.mOxyz.ys - oMaterial.position, 1.0f);
        oMaterial.mOxyz.ys = glm::vec3(newOy_s) + oMaterial.position;
        glm::vec4 newOy_e = rotationMatrix * glm::vec4(oMaterial.mOxyz.ye - oMaterial.position, 1.0f);
        oMaterial.mOxyz.ye = glm::vec3(newOy_e) + oMaterial.position;
        glm::vec4 newOz_s = rotationMatrix * glm::vec4(oMaterial.mOxyz.zs - oMaterial.position, 1.0f);
        oMaterial.mOxyz.zs = glm::vec3(newOz_s) + oMaterial.position;
        glm::vec4 newOz_e = rotationMatrix * glm::vec4(oMaterial.mOxyz.ze - oMaterial.position, 1.0f);
        oMaterial.mOxyz.ze = glm::vec3(newOz_e) + oMaterial.position;
    }

    // rotate without changing the direction of other bases
    void oMesh::rotate(float angleX, float angleY, float angleZ, const glm::vec3 pt_center) {
        glm::mat4 translationToOriginMatrix = glm::translate(glm::mat4(1.0f), -pt_center);

        glm::mat4 rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleX), glm::vec3(1, 0, 0));
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleY), glm::vec3(0, 1, 0));
        rotationMatrix = glm::rotate(rotationMatrix, glm::radians(angleZ), glm::vec3(0, 0, 1));

        glm::mat4 translationBackMatrix = glm::translate(glm::mat4(1.0f), pt_center);

        glm::mat4 finalMatrix = translationBackMatrix * rotationMatrix * translationToOriginMatrix;

        for (auto& vertex : mVertices) {
            glm::vec4 newPosition = finalMatrix * glm::vec4(vertex.mPos, 1.0f);
            vertex.mPos = glm::vec3(newPosition);
        }

        oMaterial.rotation.x += angleX;
        oMaterial.rotation.y += angleY;
        oMaterial.rotation.z += angleZ;

        glm::vec4 newCenterPosition = finalMatrix * glm::vec4(oMaterial.position, 1.0f);
        oMaterial.position = glm::vec3(newCenterPosition);

        oMaterial.mOxyz.xs = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.xs, 1.0f));
        oMaterial.mOxyz.xe = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.xe, 1.0f));
        oMaterial.mOxyz.ys = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.ys, 1.0f));
        oMaterial.mOxyz.ye = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.ye, 1.0f));
        oMaterial.mOxyz.zs = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.zs, 1.0f));
        oMaterial.mOxyz.ze = glm::vec3(finalMatrix * glm::vec4(oMaterial.mOxyz.ze, 1.0f));
    }

    void oMesh::move(float offsetX, float offsetY, float offsetZ) {
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

        // Move Oxyz
        oMaterial.mOxyz.xs.x += offsetX; oMaterial.mOxyz.xe.x += offsetX;
        oMaterial.mOxyz.ys.y += offsetY; oMaterial.mOxyz.ye.y += offsetY;
        oMaterial.mOxyz.zs.z += offsetZ; oMaterial.mOxyz.ze.z += offsetZ;
    }

    void oMesh::applyTransformation(const glm::vec3& center, Eigen::Matrix4f transform) {
        

        // Convert vertices to Eigen matrix
        Eigen::MatrixXf vtx(4, mVertices.size());
        for (size_t i = 0; i < mVertices.size(); ++i) {
            vtx(0, i) = mVertices[i].mPos.x - center.x;
            vtx(1, i) = mVertices[i].mPos.y - center.y;
            vtx(2, i) = mVertices[i].mPos.z - center.z;
            vtx(3, i) = 1.0f;
        }

        // Apply transformation to all vertices
        Eigen::MatrixXf newPositions = transform * vtx;

        // Update vertices positions
        for (size_t i = 0; i < mVertices.size(); ++i) {
            mVertices[i].mPos = glm::vec3(newPositions(0, i) + center.x, newPositions(1, i) + center.y, newPositions(2, i) + center.z);
        }

        // Update oMaterial position
        Eigen::Vector4f centerPos(oMaterial.position.x - center.x, oMaterial.position.y - center.y, oMaterial.position.z - center.z, 1.0f);
        Eigen::Vector4f newCenterPos = transform * centerPos;
        oMaterial.position.x = newCenterPos.x() + center.x;
        oMaterial.position.y = newCenterPos.y() + center.y;
        oMaterial.position.z = newCenterPos.z() + center.z;


    }

    void oMesh::calculate_normals()
    {
        // Reset normals
                #pragma omp parallel for
        for (size_t i = 0; i < mVertices.size(); ++i) {
            mVertices[i].mNormal = glm::vec3(0.0f);
        }

        // Compute normals
                #pragma omp parallel for
        for (size_t i = 0; i < mVertexIndices.size(); i += 3) {
            unsigned int i0 = mVertexIndices[i];
            unsigned int i1 = mVertexIndices[i + 1];
            unsigned int i2 = mVertexIndices[i + 2];

            glm::vec3 v0 = mVertices[i0].mPos;
            glm::vec3 v1 = mVertices[i1].mPos;
            glm::vec3 v2 = mVertices[i2].mPos;

            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 normal = glm::cross(edge1, edge2);
            float area = glm::length(normal) * 0.5f;

            normal = glm::normalize(normal);

                #pragma omp atomic
            mVertices[i0].mNormal += normal * area;

                #pragma omp atomic
            mVertices[i1].mNormal += normal * area;

                #pragma omp atomic
            mVertices[i2].mNormal += normal * area;
        }

        // Normalize the result
                #pragma omp parallel for
        for (size_t i = 0; i < mVertices.size(); ++i) {
            mVertices[i].mNormal = glm::normalize(mVertices[i].mNormal);
        }

    }

}


namespace nelems {
    std::mutex nelems::mMesh::mMutex;

    bool mMesh::load_sync(const std::string& filepath) {
        static std::ifstream file;
        static std::streampos last_pos = 0;

        if (!file.is_open()) {
            file.open(filepath, std::ios::binary);
            if (!file.is_open()) {
                return false;
            }
        }

        file.seekg(last_pos);

        std::stringstream buffer;
        buffer << file.rdbuf();

        last_pos = file.tellg();

        if (file.eof()) {
            file.close();
            last_pos = 0;
        }

        return true;
    }

    bool mMesh::load(const std::string& filepath, bool robot) {
        static glm::vec3 color[9] = {
            glm::vec3(0.1, 0.4, 0.7),
            glm::vec3(0.0, 0.5, 0.0), glm::vec3(1.0, 0.8, 0.2), glm::vec3(0.0, 0.6, 0.4),
            glm::vec3(0.2, 0.1, 0.1), glm::vec3(0.04, 0.04, 0.4), glm::vec3(0.4, 0.1, 0.7)
        };
        const uint32_t cMeshImportFlags =
            aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_OptimizeMeshes | aiProcess_JoinIdenticalVertices|
            aiProcess_ValidateDataStructure;

        Assimp::Importer Importer;

        const aiScene* pScene = Importer.ReadFile(filepath.c_str(), cMeshImportFlags);

        std::string filename = std::filesystem::path(filepath).filename().stem().string() + " ";

        if (pScene && pScene->HasMeshes()) {
            static std::map<std::string, glm::vec3> ctMap;

            if (robot) {
                for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                    auto* mesh = pScene->mMeshes[i];
                    std::string name = mesh->mName.C_Str();
                    if (name.find("RBSIMCenter_") != std::string::npos) {
                        ctMap[name] = glm::vec3(mesh->mVertices[0].x, mesh->mVertices[0].y, mesh->mVertices[0].z);
                    }
                }
            }

            static int color_idx = 0;

            for (unsigned int i = 0; i < pScene->mNumMeshes; ++i) {
                auto* mesh = pScene->mMeshes[i];
                oMesh newMesh;
                newMesh.ID = getCurrentTimeMillis(pScene->mNumMeshes);
                load_specific_mesh(mesh, newMesh);
                if (robot) {
                    std::string name = mesh->mName.C_Str();
                    // Center Point => Skip
                    if (name.find("RBSIMCenter_") != std::string::npos) { continue; }
                    // Table
                    else if (name.find("table") != std::string::npos)
                    {
                        newMesh.oMaterial.mMetallic = 0; newMesh.oMaterial.mRoughness = 0;
						newMesh.changeName(name);
						newMesh.oMaterial.mColor = glm::vec3(0.2f, 0.2f, 0.2f);
                    }
                    // Base
                    else
                    {
                        newMesh.changeName(name);
                        newMesh.oMaterial.mColor = color[color_idx++];
                        name = "RBSIMCenter_" + name.substr(name.find_last_of("_") + 1);
                        newMesh.oMaterial.position = ctMap[name];
                    }
                }
                else {
                    newMesh.changeName(filename + std::to_string(newMesh.ID % 1000));
                }
                newMesh.oMaterial.mOxyz.xs = newMesh.oMaterial.position;
                newMesh.oMaterial.mOxyz.xe = newMesh.oMaterial.position + glm::vec3(1000.0f, 0.0f, 0.0f);
                newMesh.oMaterial.mOxyz.ys = newMesh.oMaterial.position;
                newMesh.oMaterial.mOxyz.ye = newMesh.oMaterial.position + glm::vec3(0.0f, 1000.0f, 0.0f);
                newMesh.oMaterial.mOxyz.zs = newMesh.oMaterial.position;
                newMesh.oMaterial.mOxyz.ze = newMesh.oMaterial.position + glm::vec3(0.0f, 0.0f, 1000.0f);
                mMeshes->push_back(std::make_shared<oMesh>(newMesh));

            }
            return true;
        }
        return false;
    }

    void mMesh::load_specific_mesh(const aiMesh* mesh, oMesh& outMesh) {
        for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
            VertexHolder vh;
            vh.mPos = { mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z };
            vh.mNormal = { mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z };
            outMesh.mVertices.push_back(vh);
        }

        for (size_t i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            for (size_t j = 0; j < face.mNumIndices; j++) {
                outMesh.mVertexIndices.push_back(face.mIndices[j]);
            }
        }
        glm::vec3 center = glm::vec3(0.0f);
        for (const auto& vertex : outMesh.mVertices) {
            center += vertex.mPos;
        }
        center /= static_cast<float>(outMesh.mVertices.size());
        outMesh.oMaterial.position = center;
        outMesh.init();
    }

    long long mMesh::getCurrentTimeMillis(int size) {
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

        do {
            auto now = std::chrono::system_clock::now();
            millis++;
            bool found = false;
            for (const auto& mesh : *mMeshes) {
                if (mesh->ID == millis) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                uniqueIDFound = true;
            }
        } while (!uniqueIDFound);

        return millis;
    }

    void mMesh::clear_meshes() {
        if (mMeshes == nullptr) return;
        for (auto& mesh : *mMeshes) {
            mesh->delete_buffers();
        }
        mMeshes->clear();
        if (mCoorSystem == nullptr) return;
        mCoorSystem->delete_buffers();
        mCoorSystem.reset();
    }

    void mMesh::update(nshaders::Shader* shader, int lightmode) {
        shader->set_i1(lightmode, "LightModes");
        for (auto& mesh : *mMeshes) {
            shader->set_material(mesh->oMaterial, "materialData");
            mesh->render();
            mesh->unbind();
        }
        for (auto& mesh : *mMeshes) {
            // incase robot
            if (std::string(mesh->oname).find("movepath__SKIP__") != std::string::npos)
            {
                shader->set_material(mesh->oMaterial, "materialData");
                mesh->render_lines();
                mesh->unbind();
                continue;
            }
            // incase selected
            if (!mesh->selected) { continue; }
            oMesh OBxyz, OBox, OBoy, OBoz;
            set_OBxyz(axis_length, *mesh, OBox, OBoy, OBoz);
            shader->set_material(OBox.oMaterial, "materialData");
            OBox.render_lines();    shader->set_material(OBoy.oMaterial, "materialData");
            OBoy.render_lines();    shader->set_material(OBoz.oMaterial, "materialData");
            OBoz.render_lines();
            OBox.unbind(); OBoy.unbind(); OBoz.unbind();
        }
        shader->set_i1(2, "LightModes");
        shader->set_material(mCoorSystem->oMaterial, "materialData");
        mCoorSystem->render_lines();
		mCoorSystem->unbind();
    }

    void mMesh::createGridSys(float gridNo, float step) {
        float vlue;
        if (!mCoorSystem) {
            int mini_no = 2;
            mCoorSystem = std::make_unique<oMesh>();
            mCoorSystem->changeName("Coordinate System");

            mCoorSystem->oMaterial.mColor = glm::vec3(0.8f, 0.7f, 0.75f);
            mCoorSystem->ID = getCurrentTimeMillis(0);

            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->add_vertex(vertex);
                    if (x < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f + 1.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
                    if (y < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
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
            for (float x = -gridNo / 2.0f; x < gridNo / 2.0f; x++) {
                for (float y = -gridNo / 2.0f; y < gridNo / 2.0f; y++) {
                    VertexHolder vertex;
                    vertex.mPos = { x * step, y * step, 0.0f };
                    vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                    mCoorSystem->add_vertex(vertex);
                    if (x < gridNo / 2.0f - 1) {
                        vlue = (x + gridNo / 2.0f) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                        vlue = (x + gridNo / 2.0f + 1) * gridNo + (y + gridNo / 2.0f);
                        mCoorSystem->add_vertex_index(static_cast<unsigned int>(vlue));
                    }
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

    std::shared_ptr<nelems::oMesh> mMesh::get_mesh_ptr(int j) {
        if (j < 0 || j >= mMeshes->size()) {
            return nullptr;
        }
        return (*mMeshes)[j];
    }

    void mMesh::get_mesh_ptr(int& j, std::shared_ptr<nelems::oMesh>& mesh) {
        if (j < 0 || j >= mMeshes->size()) {
            mesh = nullptr;
            return;
        }
        mesh = (*mMeshes)[j];
    }

    void mMesh::get_mesh_ptr(long long ids, std::shared_ptr<nelems::oMesh>& mesh) {
        for (const auto& m : *mMeshes) {
            if (m->ID == ids) {
                mesh = m;
                return;
            }
        }
        mesh = nullptr;
    }

    bool mMesh::get_mesh_byname(const std::string& name, std::shared_ptr<nelems::oMesh>& mesh)
    {
		for (const auto& m : *mMeshes) {
			if (std::string(m->oname).find(name) != std::string::npos) {
				mesh = m;
				return true;
			}
		}
		mesh = nullptr;
        return false;
    }

    void mMesh::delete_selected() {
        for (auto& mesh : *mMeshes) {
            if (mesh->selected) {
                if (std::string(mesh->oname).find("RBSIMBase_") != std::string::npos) { continue; }
                if (std::string(mesh->oname).find("__SKIP__") != std::string::npos) { continue; }
                mesh->delete_buffers();
            }
        }
        mMeshes->erase(std::remove_if(mMeshes->begin(), mMeshes->end(),
            [](const std::shared_ptr<oMesh>& mesh) { return mesh->selected; }), mMeshes->end());
    }

    void mMesh::delete_byname(const std::string& delmesh)
    {
        auto new_end = std::remove_if(mMeshes->begin(), mMeshes->end(),
            [&](const std::shared_ptr<oMesh>& mesh) {
                if (std::string(mesh->oname).find(delmesh) != std::string::npos)
                {
                    mesh->delete_buffers();
                    return true; 
                }
                return false;
            });

        mMeshes->erase(new_end, mMeshes->end());
    }



    void mMesh::add_mesh(std::shared_ptr<oMesh> addnewmesh)
    {
        mMeshes->push_back(addnewmesh);
    }

    void mMesh::set_OBxyz(float length, oMesh& mesh, oMesh& OBox, oMesh& OBoy, oMesh& OBoz) {
        /*oMesh oxMesh;
        oxMesh.mVertices.clear();
        oxMesh.mVertexIndices.clear();
        oxMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.xs, glm::vec3(1.0f, 0.0f, 0.0f)));
        oxMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.xe, glm::vec3(1.0f, 0.0f, 0.0f)));
        oxMesh.add_vertex_index(0);
        oxMesh.add_vertex_index(1);
        oxMesh.init();
        OBox = oxMesh;
        OBox.oMaterial.mColor = glm::vec3(1.0f, 0.0f, 0.0f);
        oMesh oyMesh;
        oyMesh.mVertices.clear();
        oyMesh.mVertexIndices.clear();
        oyMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.ys, glm::vec3(0.0f, 1.0f, 1.0f)));
        oyMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.ye, glm::vec3(0.0f, 1.0f, 1.0f)));
        oyMesh.add_vertex_index(0);
        oyMesh.add_vertex_index(1);
        oyMesh.init();
        OBoy = oyMesh;
        OBoy.oMaterial.mColor = glm::vec3(0.0f, 1.0f, 1.0f);
        oMesh ozMesh;
        ozMesh.mVertices.clear();
        ozMesh.mVertexIndices.clear();
        ozMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.zs, glm::vec3(0.5f, 0.0f, 0.5f)));
        ozMesh.add_vertex(VertexHolder(mesh.oMaterial.mOxyz.ze, glm::vec3(0.5f, 0.0f, 0.5f)));
        ozMesh.add_vertex_index(0);
        ozMesh.add_vertex_index(1);
        ozMesh.init();
        OBoz = ozMesh;
        OBoz.oMaterial.mColor = glm::vec3(0.5f, 0.0f, 0.5f);*/
        
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
