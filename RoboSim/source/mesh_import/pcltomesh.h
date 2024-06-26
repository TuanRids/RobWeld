#pragma once
#include "pch.h"
#include "elems/mesh.h"
#include <vector>
#include <string>

class PclToMesh {
public:
    PclToMesh() {}
    void processPointCloud();
    void addToMesh();
    void loadDataCrXyz();

private:
    std::vector<std::array<float,3>> ptCloud; // data format [rows][cols][3]
    std::string filePath = "dataraw2.txt"; // RawData_Sang.txt
    nelems::oMesh oMeshObject;

    void loadPointCloud();
    void filterZeroValues();
    void createMesh();
    void createMesh_pcl();
};
