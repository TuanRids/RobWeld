#pragma once
#include "pch.h"
#include <stdexcept>
#include <chrono>
#include "elems/mesh.h"
#include "ui/statuslogs.h"
#include "Filemgr/RobInitFile.h"

struct Point {
    float x, y, z;
};

struct Triangle {
    Point p1, p2, p3;
};


class PclToMesh {
public:

    PclToMesh() { proMesh = &nelems::mMesh::getInstance(); sttlogs = &nui::StatusLogs::getInstance(); }
    void processPointCloud(int keyID);
    void setter_data(const std::vector<std::vector<float>>& setterdata) { data = setterdata; }
    void setter_creating_speed(const unsigned int& speed) { creating_speed = speed; }
private:
    size_t key = 0;
    RobInitFile* robinit;
    unsigned int creating_speed = 0;
    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm
    std::string filePath = Config::PATH_TO_3D_HUNG;
    nelems::mMesh* proMesh;
    nui::StatusLogs* sttlogs;
    std::vector<std::vector<float>> data;
    void logTime(std::chrono::time_point<std::chrono::high_resolution_clock>& start, const std::string& message);
};