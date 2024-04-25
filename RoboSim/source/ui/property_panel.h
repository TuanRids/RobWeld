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
#include "FrameManage.h"
#include "Command/cmdManage.h"
#include "Command/MoveOb.h"


namespace nui
{
    /*
    Property Panel Use to manage the Properties of the Mesh
    */
    class Property_Panel 
    {
    private:
        // Transformation
        ncommand::ObHistory &obAction;
        nelems::mMesh* proMesh; // Mesh Properties
        nelems::oMesh* mesh = nullptr; // for each objects
        long long selectedID = 0;


        // create a file browser instance
        ImGui::FileBrowser mFileDialog;
        std::function<void(const std::string&)> mMeshLoadCallback;
        std::string mCurrentFile;
        public:
            Property_Panel(ncommand::ObHistory& crobHistory):
                proMesh(nullptr),mesh(nullptr), obAction(crobHistory)
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
            void layer_frame(nui::SceneView* scene_view, std::vector<long long> &IDs);
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


