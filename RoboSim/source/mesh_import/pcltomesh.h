#pragma once
#include "pch.h"
#include <stdexcept>
#include <chrono>
#include "elems/mesh.h"
#include "ui/statuslogs.h"
#include "Filemgr/RobInitFile.h"
#include "ui/scene_view.h"
#include "cudacal/cudacal.h"



class PclToMesh {
public:

    PclToMesh();
    void setter_data(const std::vector<std::vector<float>>& setterdata) { data = setterdata; }
    void setter_creating_speed(const unsigned int& speed) { creating_speed = speed; }

    void show3d_data();
    void reset_unqueIDpath(){datpath = ""; }
private:
    std::string datpath;
    RobInitFile* robinit;
    unsigned int creating_speed = 0;
    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm
    nelems::mMesh* proMesh;
    nui::StatusLogs* sttlogs;
    nui::SceneView* sceneview;
    std::vector<std::vector<float>> data;
    void logTime(std::chrono::time_point<std::chrono::high_resolution_clock>& start, const std::string& message);

};