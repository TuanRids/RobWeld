#pragma once

#include "ui/scene_view.h"
#include <string>

#include <filesystem>

#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include <Windows.h>
// ==================LOAD ROBOT====================

namespace fs = std::filesystem;

class LoadRobot
{
private:
    nui::SceneView* scene_view;
    std::string mCurrentFile;

public:
    LoadRobot() : scene_view(nullptr) {
		scene_view = &nui::SceneView::getInstance();
	}
    void trigger_GP8()
    {
        fs::path currentDir = fs::current_path();

        fs::path filePath = currentDir / "RobotStandard\\Yaskawa-gp8-113k.fbx";

        std::string filePathStr = filePath.string();

        scene_view->load_mesh(filePathStr);
    }
};