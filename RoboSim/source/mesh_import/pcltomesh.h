#pragma once
#include "pch.h"
#include <Eigen/Dense>
#include <stdexcept>
#include <chrono>
#include "elems/mesh.h"
#include "ui/statuslogs.h"
#include <thread>



class PclToMesh {
public:

    PclToMesh() { proMesh = &nelems::mMesh::getInstance(); sttlogs = &nui::StatusLogs::getInstance(); }
    void processPointCloud();

private:

    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm

    std::string filePath = Config::PATH_TO_3D_HUNG;

    void Create3DPCL();
    nelems::mMesh* proMesh;
    nui::StatusLogs* sttlogs;
};