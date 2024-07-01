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

#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include "Eigen/Dense"

#include "py3rdsrc/zmpdata.h"


#include <Windows.h>
#include "render/ui_context.h"
#include "ymrobot/ymconnect.h"
#include "statuslogs.h"
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
        std::shared_ptr<nelems::oMesh> mesh;
        std::vector<std::shared_ptr<nelems::oMesh>> base;
        nui::uiAction uiaction;
        std::unordered_set<long long> selectedMeshes;
        std::shared_ptr<nshaders::Shader> mctshader;
        float an1{ 0 }, an2{ 0 }, an3{ 0 }, an4{ 0 }, an5{ 0 }, an6{ 0 };
        nymrobot::ymconnect* mRobot;
        StatusLogs* sttlogs;
        std::unique_ptr<zmpdata> IPreceiver;

        bool CtrFlag = false; // Livesync & visualize
    public:
        Property_Panel():
            proMesh(nullptr),mesh(nullptr), mRobot(nullptr), mctshader(nullptr), sttlogs(nullptr), IPreceiver(nullptr)
        {
            for (int i{ 0 }; i < 7; i++) { base.push_back(nullptr); }
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
            IPreceiver = std::make_unique<zmpdata>();
        }
        void render(nui::SceneView* mScene, GLFWwindow* mWindow);
        void material_frame(nui::SceneView* scene_view);
        void camera_frame(nui::SceneView* scene_view);
        void layer_frame(nui::SceneView* scene_view);
        void Robot_Controls_table();
        void rotateJoint(size_t jointIndex, float& ang, float& pre, const float tolerance,
            std::vector<std::shared_ptr<nelems::oMesh>>& base,
            float diffX, float diffY, float diffZ);
        void obInfo_frame();
        void coordinate_frame();
        bool check_skip(const char(&name)[256]);
        void sh_performance();
        void draft_chart();
        /// mode: dark, light

        ~Property_Panel() { 
            mesh = nullptr;
            mesh = nullptr;
            proMesh = nullptr;
        }
        
        void SaveIniFile(const std::string& key, const std::string& value);
        void SwitchVisualLiveSync(){ CtrFlag = !CtrFlag; }
      };
}


