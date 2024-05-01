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

#include <unordered_set>
#include "nlohmann/json.hpp"
#include <Windows.h>
#include "render/ui_context.h"

using json = nlohmann::json;

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
        //nui::uiAction uiaction;
        std::unordered_set<long long> selectedMeshes;



    public:
        Property_Panel():
            proMesh(nullptr),mesh(nullptr)
        {
            std::string content = "Arial"; std::ifstream file("robosim_ini.dat");
            if (file.is_open())
            {
                json j; file >> j;
                if (j.find("font") != j.end()) {
                    content = j["font"].get<std::string>();
                }
            }
            ImGuiIO& io = ImGui::GetIO();
            std::string fontPath = "C:/Windows/Fonts/" + std::string(content) + ".ttf";
            io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 16.0f);
        }

        void render(nui::SceneView* mScene, GLFWwindow* mWindow);
        void material_frame(nui::SceneView* scene_view);
        void camera_frame(nui::SceneView* scene_view);
        void layer_frame(nui::SceneView* scene_view);
        void obInfo_frame();
        void coordinate_frame();

        /// mode: dark, light

        ~Property_Panel() { 
            mesh = nullptr;
            mesh = nullptr;
            proMesh = nullptr;
        }
        
        void SaveIniFile(const std::string& key, const std::string& value);
      };
}


