#pragma once
#include "pch.h"
#include "scene_view.h"
#include "ui/uiAction.h"
#include "render/ui_context.h"
#include "mesh_import/pcltomesh.h"
#include "mutex"
#include "Filemgr/RobInitFile.h"
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
        RobInitFile* robinit;
        static bool waitloop[6];
        static bool shint;
        std::string theme;
        std::string rob_font;
        std::mutex mtx;

        bool OptionSetting_Flag = false;
        void OptionSettings();
        void OpenFileDialog();
        void hint(bool show);

        void OpenFontDialog(HWND owner);

    public:
        HotkeyMenubar() : scene_view(nullptr){
            scene_view = &nui::SceneView::getInstance();
            robinit = &RobInitFile::getinstance();
        }
        ////====    ==============MENU BAR==================
        //void commandLogs(){  }
        void mMenuBar(GLFWwindow* mWindow);
        void mHotkey(GLFWwindow* mWindow);
                

        ////====    ============== CONNECT TO ROBOT====================

	};
}