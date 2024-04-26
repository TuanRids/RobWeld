#pragma once
#include "pch.h"
#include "elems/light.h"
#include "ui/scene_view.h"
#include "elems/mesh.h"

#include "imgui.h"
#include "utils/imgui_widgets.h"
#include <ImFileBrowser.h>
#include <Windows.h>
#include <Commdlg.h>
#include "utils/RobsFileIO.h"
#include "ui/FrameManage.h"
#include "ui/uiAction.h"
#include <unordered_set>

namespace nui
{
    /*
    Property Panel Use to manage the Properties of the Mesh
    TAKECARE: This class is not a singleton class
    IT COULD MAKE MANY INSTANCE OF THIS CLASS CAUSE THE MEMORY LEAK AND BUGS
    ------------------------------------------
    */
    class Property_Panel 
    {
    private:
        // Transformation
        nelems::mMesh* proMesh; // Mesh Properties
        nelems::oMesh* mesh = nullptr; // for each objects
        nui::uiAction uiaction;
        std::unordered_set<long long> selectedMeshes;

        //

        // create a file browser instance
        ImGui::FileBrowser mFileDialog;
        std::function<void(const std::string&)> mMeshLoadCallback;
        std::string mCurrentFile;
    public:
        Property_Panel():
            proMesh(nullptr),mesh(nullptr)
        {
                
            mCurrentFile = "< ... >";
            mFileDialog.SetTitle("Import Mesh");
            mFileDialog.SetFileFilters({ ".fbx", ".obj",".stl"});
            ImGuiIO& io = ImGui::GetIO();
            io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/Arial.ttf", 16.0f);

        }

        void render(nui::SceneView* mScene);
        void material_frame(nui::SceneView* scene_view);
        void camera_frame(nui::SceneView* scene_view);
        void layer_frame(nui::SceneView* scene_view);
        void obInfo_frame();
        void coordinate_frame();

        ~Property_Panel() { 
            mesh = nullptr;
            mesh = nullptr;
            proMesh = nullptr; 
        }
        void SetMeshLoadCallback(const std::function<void(const std::string&)>& callback)
                        { mMeshLoadCallback = callback;  }
        void MenuBar();
        void OpenFileDialog();
        
      };
}


