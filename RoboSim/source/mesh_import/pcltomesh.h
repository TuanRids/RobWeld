#pragma once
#include "pch.h"
#include <stdexcept>
#include <chrono>
#include "elems/mesh.h"
#include "ui/statuslogs.h"




class PclToMesh {
public:

    PclToMesh() { proMesh = &nelems::mMesh::getInstance(); sttlogs = &nui::StatusLogs::getInstance(); }
    void processPointCloud();
    void setter_data(const std::vector<std::vector<float>>& setterdata) { data = setterdata; }
private:

    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm

    std::string filePath = Config::PATH_TO_3D_HUNG;

    void Create3DPCL(const float &SizeLeaf, const unsigned int &poidepth);
    nelems::mMesh* proMesh;
    nui::StatusLogs* sttlogs;
    std::vector<std::vector<float>> data;
};