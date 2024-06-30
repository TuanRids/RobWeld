#pragma once
#include "pch.h"
#include "elems/mesh.h"
#include <vector>
#include <string>
#include <set>


// TODO
// - Segmentation
// - reduce analysis time
// - export obj [or png topviews]
// 
//
class PclToMesh {
public:
    PclToMesh() {}
    void processPointCloud();
    void addToMesh();
    void loadDataCrXyz();

private:
    std::vector<std::array<float,3>> ptCloud; // data format [rows][cols][3]
    size_t num_rows;               /* some appropriate calculation */ // You need to define how num_rows is computed


    std::string filePath =  Config::PATH_TO_3D_HUNG ; // Sang dataclean.txt  Hung RawData_2
    nelems::oMesh oMeshObject;
    nelems::oMesh segmentedObject;
    void segmentPointCloud();
    // void loadPointCloud();
    void filterZeroValues();
    void createMesh();
    void createMesh_pcl();

    // Helper functions
    float length(const glm::vec3& vec) {
        return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    }
    float distance(const glm::vec3& vec1, const glm::vec3& vec2) {
        return length(vec1 - vec2);
    }

};
