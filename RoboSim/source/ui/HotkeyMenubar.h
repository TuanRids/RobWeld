#pragma once
#include "pch.h"
#include "ui/scene_view.h"
#include "ui/uiAction.h"
#include "render/ui_context.h"
#include "mesh_import/pcltomesh.h"
#include "mutex"
#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include <ymrobot/ymconnect.h>
using json = nlohmann::json;
namespace nui {
    // Manage the hotkey and menubar
    // all actions will move to uiAction
	class HotkeyMenubar {
    private:
        uiAction uiaction;
        nui::SceneView* scene_view;

        std::string mCurrentFile;
        static bool waitloop[6];
        static bool shint;
        std::string theme;
        nymrobot::ymconnect* mRobot;
        std::mutex mtx;
    public:
        HotkeyMenubar() : scene_view(nullptr), mRobot(nullptr){
            scene_view = &nui::SceneView::getInstance();
            
        }
        ////====    ==============MENU BAR==================
        void commandLogs(){ uiaction.Command_Logs(); }
        void mMenuBar(GLFWwindow* mWindow);
        
        void mHotkey(GLFWwindow* mWindow);

        ////====    ==============OPEN FILE DIALOG==========
        void SaveIniFile(const std::string& key, const std::string& value);

        void OpenFileDialog();
        
        void hint(bool show);

        ////====    ============== CONNECT TO ROBOT====================

	};
}