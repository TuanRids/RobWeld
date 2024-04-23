#include <pch.h>
#include <vector>
#include "elems/material.h" // Include your Material definition here
#include "elems/vertex_holder.h" // Include your VertexHolder definition here
#include <fstream>
#include <iostream>
#include "elems/mesh.h"
#include "imgui.h"
#include <ImFileBrowser.h>
#include <Windows.h>
#include <Commdlg.h>
namespace nelems
{

    class RobsFileIO 
    {
 
    public:
        
        static bool SaveToFile(nelems::mMesh& mesh)
        {
            std::string filePath;
            //--------------------------------------------------------------------------------
            /// Files Dialog 
            OPENFILENAME ofn;
            char szFile[260] = { 0 };

            ZeroMemory(&ofn, sizeof(ofn));
            ofn.lStructSize = sizeof(ofn);
            ofn.hwndOwner = NULL;
            ofn.lpstrFile = szFile;
            ofn.nMaxFile = sizeof(szFile);
            ofn.lpstrFilter = "ROBS files (*.robs)\0*.robs\0All Files (*.*)\0*.*\0";
            ofn.nFilterIndex = 1;
            ofn.lpstrFileTitle = NULL;
            ofn.nMaxFileTitle = 0;
            ofn.lpstrInitialDir = NULL;
            ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

            if (GetSaveFileName(&ofn) == TRUE)
            {
                filePath = std::string(ofn.lpstrFile);
                size_t robsIndex = filePath.find(".robs");
                if (robsIndex == std::string::npos)
				{
					filePath += ".robs";
				}
            }

            else
            {
                return false;
            }
            // check:
            
            //--------------------------------------------------------------------------------
            //// write Data
            std::ofstream file(filePath, std::ios::binary);

            if (!file.is_open()) {
                std::cerr << "Failed to open file for writing: " << filePath << std::endl;
                return false;
            }

            uint32_t numMeshes = static_cast<uint32_t>(mesh.size());
            file.write(reinterpret_cast<char*>(&numMeshes), sizeof(numMeshes));

            for (int i = 0; i < mesh.size(); ++i)
            {
                nelems::oMesh* oMesh;
                mesh.get_mesh_ptr(i, oMesh);

                file.write(reinterpret_cast<char*>(&oMesh->ID), sizeof(oMesh->ID));

                uint32_t numVertices = static_cast<uint32_t>(oMesh->mVertices.size());
                file.write(reinterpret_cast<char*>(&numVertices), sizeof(numVertices));
                file.write(reinterpret_cast<const char*>(oMesh->mVertices.data()), numVertices * sizeof(VertexHolder));

                uint32_t numIndices = static_cast<uint32_t>(oMesh->mVertexIndices.size());
                file.write(reinterpret_cast<char*>(&numIndices), sizeof(numIndices));
                file.write(reinterpret_cast<const char*>(oMesh->mVertexIndices.data()), numIndices * sizeof(unsigned int));

                // Write material data
                file.write(reinterpret_cast<const char*>(&oMesh->oMaterial.mColor), sizeof(oMesh->oMaterial.mColor));
                file.write(reinterpret_cast<const char*>(&oMesh->oMaterial.metallic), sizeof(oMesh->oMaterial.metallic));
                file.write(reinterpret_cast<const char*>(&oMesh->oMaterial.roughness), sizeof(oMesh->oMaterial.roughness));
                file.write(reinterpret_cast<const char*>(&oMesh->oMaterial.ao), sizeof(oMesh->oMaterial.ao));

                std::cout << oMesh->ID << std::endl;
            }

            return true;
        }

        static bool LoadFromFile(nelems::mMesh& mesh) {
            //--------------------------------------------------------------------------------
            /// Files Dialog
            std::string filePath;
            OPENFILENAME ofn;
            char szFile[260] = { 0 };

            ZeroMemory(&ofn, sizeof(ofn));
            ofn.lStructSize = sizeof(ofn);
            ofn.hwndOwner = NULL;
            ofn.lpstrFile = szFile;
            ofn.nMaxFile = sizeof(szFile);
            ofn.lpstrFilter = "ROBS files (*.robs)\0*.robs\0All Files (*.*)\0*.*\0";
            ofn.nFilterIndex = 1;
            ofn.lpstrFileTitle = NULL;
            ofn.nMaxFileTitle = 0;
            ofn.lpstrInitialDir = NULL;
            ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

            if (GetOpenFileName(&ofn) == TRUE)
            {
                filePath = std::string(ofn.lpstrFile);
            }
            else {
                return false;
            }
            //--------------------------------------------------------------------------------
            //// read Data
            std::ifstream file(filePath, std::ios::binary);
            if (!file.is_open()) {
                std::cerr << "Failed to open file for reading: " << filePath << std::endl;
                return false;
            }
            uint32_t numMeshes;
            file.read(reinterpret_cast<char*>(&numMeshes), sizeof(numMeshes));
            
            for (int i = 0; i < numMeshes; ++i)
            {
                std::cout << "Loading mesh " << i << std::endl;
                nelems::oMesh oMesh;
                // Read mesh ID
                file.read(reinterpret_cast<char*>(&oMesh.ID), sizeof(oMesh.ID));

                // Read vertices
                uint32_t numVertices;
                file.read(reinterpret_cast<char*>(&numVertices), sizeof(numVertices));
                oMesh.mVertices.resize(numVertices);
                file.read(reinterpret_cast<char*>(oMesh.mVertices.data()), numVertices * sizeof(VertexHolder));

                // Read indices
                uint32_t numIndices;
                file.read(reinterpret_cast<char*>(&numIndices), sizeof(numIndices));
                oMesh.mVertexIndices.resize(numIndices);
                file.read(reinterpret_cast<char*>(oMesh.mVertexIndices.data()), numIndices * sizeof(unsigned int));

                // Read material
                file.read(reinterpret_cast<char*>(&oMesh.oMaterial.mColor), sizeof(oMesh.oMaterial.mColor));
                file.read(reinterpret_cast<char*>(&oMesh.oMaterial.metallic), sizeof(oMesh.oMaterial.metallic));
                file.read(reinterpret_cast<char*>(&oMesh.oMaterial.roughness), sizeof(oMesh.oMaterial.roughness));
                file.read(reinterpret_cast<char*>(&oMesh.oMaterial.ao), sizeof(oMesh.oMaterial.ao));
                oMesh.init();
                // Add the loaded mesh to the current mesh
                mesh.pushback(oMesh);
            }
            return true;
        }
    };
}
