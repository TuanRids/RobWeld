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

#include "ui/uiAction.h"
#include <unordered_set>

#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include "Eigen/Dense"

#include "Filemgr/RobInitFile.h"

#include <Windows.h>
#include "render/ui_context.h"
#include "ymrobot/ymconnect.h"
#include "statuslogs.h"
using json = nlohmann::json;
#include "IUIComponent.h"
#include "CMDReader.h"
namespace nui
{
    class Property_Panel : public IUIComponent
    {
    private:
        std::unique_ptr<nui::CMDReader> cmdrder;

        nui::uiAction uiaction;
        nymrobot::ymconnect* mRobot;
        nui::StatusLogs* sttlogs;
        RobInitFile* robinit;

        float an1{ 0 }, an2{ 0 }, an3{ 0 }, an4{ 0 }, an5{ 0 }, an6{ 0 };
        bool CtrFlag = false; // Livesync & visualize
        void resize(int32_t width, int32_t height) override {}
        void material_frame();
        void camera_frame();
        void layer_frame();        
        void obInfo_frame();
        void coordinate_frame();
        bool check_skip(const std::shared_ptr<nelems::oMesh>& mesh);
        void sh_performance();
    public:
        Property_Panel();
        void render() override {}
        void render(GLFWwindow* mWindow);
        ~Property_Panel() { if (cmdrder) { cmdrder->CMDClear(); } }
       
      };
}


