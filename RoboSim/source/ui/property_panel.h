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
#include <filesystem> // For std::filesystem
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
#include "Filemgr/RobInitFile.h"

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
        std::unique_ptr<nymrobot::ymconnect> mRobot;
        nui::StatusLogs* sttlogs;
        std::unique_ptr<zmpdata> IPreceiver;
        RobInitFile* robinit;

        float an1{ 0 }, an2{ 0 }, an3{ 0 }, an4{ 0 }, an5{ 0 }, an6{ 0 };
        bool CtrFlag = false; // Livesync & visualize
    public:
        Property_Panel():
            proMesh(nullptr),mesh(nullptr), mRobot(nullptr), mctshader(nullptr), sttlogs(nullptr), IPreceiver(nullptr)
        {

            for (int i{ 0 }; i < 7; i++) { base.push_back(nullptr); }
            robinit = &RobInitFile::getinstance();
            ImGuiIO& io = ImGui::GetIO();
            std::string fontPath; robinit->get_settings("rob_font", fontPath);
            fontPath = "RobFonts\\" + fontPath + ".ttf";
            // Check if fontPath existed
            if (std::filesystem::exists(fontPath) && fontPath.size() > 10 ) 
            {io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 16.0f); }

            IPreceiver = std::make_unique<zmpdata>();
            mRobot = std::make_unique<nymrobot::ymconnect>();
            sttlogs = &nui::StatusLogs::getInstance();
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
        bool check_skip(const std::shared_ptr<nelems::oMesh>& mesh);
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


