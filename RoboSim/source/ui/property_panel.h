#pragma once

#include "elems/light.h"
#include "ui/scene_view.h"
#include "elems/mesh.h"

#include "imgui.h"
#include "utils/imgui_widgets.h"
#include <ImFileBrowser.h>
#include <Windows.h>
#include <Commdlg.h>
#include "utils/RobsFileIO.h"

/*
* Property Panel Use to manage the Properties of the Mesh
* 
*/

namespace nui
{
    class Property_Panel 
    {
    private:
        nelems::mMesh* proMesh; // Mesh Properties
        // nui::SceneView* mSceneMesh; // scene for lights, camera and others
        nelems::oMesh* mesh = nullptr; // for each objects
        long long selectedID = 0;
        bool showMenuBar;
        // create a file browser instance
        ImGui::FileBrowser mFileDialog;
        std::function<void(const std::string&)> mMeshLoadCallback;
        std::string mCurrentFile;
        public:
            Property_Panel()
            {
                mCurrentFile = "< ... >";
                mFileDialog.SetTitle("Import Mesh");
                mFileDialog.SetFileFilters({ ".fbx", ".obj",".stl"});
                ImGuiIO& io = ImGui::GetIO();
                io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/Arial.ttf", 16.0f);
                showMenuBar = true;
            }

            void render(nui::SceneView* mScene);
            ~Property_Panel() { 
                delete mesh; 
                mesh = nullptr;
                proMesh = nullptr; }
            void SetMeshLoadCallback(const std::function<void(const std::string&)>& callback)
                            { mMeshLoadCallback = callback;  }
            void MenuBar();
            void OpenFileDialog();
        
      };
}


