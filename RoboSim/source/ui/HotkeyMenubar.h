#pragma once
#include "pch.h"
#include "scene_view.h"
#include "ui/uiAction.h"
#include "render/ui_context.h"
#include "mutex"
#include "Filemgr/RobInitFile.h"
#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include <ymrobot/ymconnect.h>
#include "IPCTransfer/IPCtransfer.h"

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
        std::string rob_font="arial"; // verdanab seguihis arial
        std::mutex mtx;
        IPCtransfer* ipc;
        bool OptionSetting_Flag = false;
        void OptionSettings();
        void OpenFileDialog();
        void hint(bool show);
        nelems::mMesh* proMesh;
        void OpenFontDialog(HWND owner);

    public:
        HotkeyMenubar() : scene_view(nullptr),ipc(nullptr), proMesh(nullptr){
            scene_view = &nui::SceneView::getInstance();
            robinit = &RobInitFile::getinstance();
            proMesh = &nelems::mMesh::getInstance();
        }
        ////====    ==============MENU BAR==================
        //void commandLogs(){  }
        void mMenuBar(GLFWwindow* mWindow);
        void mHotkey(GLFWwindow* mWindow);              

	};
}