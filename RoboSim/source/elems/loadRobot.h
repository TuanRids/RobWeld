#pragma once

#include "ui/scene_view.h"
#include "elems/mesh.h"
#include <string>

#include <filesystem>
#include "pch.h"
#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
#include <Windows.h>
// ==================LOAD ROBOT====================
#include "cfreader.h"
namespace fs = std::filesystem;

class LoadRobot
{
private:
    nui::SceneView* scene_view;
    std::string mCurrentFile;
    nelems::mMesh* mMesh;
public:
    LoadRobot() : scene_view(nullptr) {
		scene_view = &nui::SceneView::getInstance();
        mMesh = &nelems::mMesh::getInstance();
	}
    void trigger_GP8()
    {
        fs::path currentDir = fs::current_path();

        fs::path filePath = currentDir / "RobotStandard//Yaskawa-gp8-113k.fbx";            //Config::PATH_TO_ROBOTEMPLATE;

        std::string filePathStr = filePath.string();

        scene_view->load_mesh(filePathStr,1);
        
    }
};