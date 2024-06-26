#define _CRT_SECURE_NO_WARNINGS
#include "pch.h"
#include "pclToMesh.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <cmath>
#include <pcl/features/normal_3d.h>


void PclToMesh::filterZeroValues() {

    std::vector<std::vector<std::vector<float>>> data; // data format [rows][cols][3]
    // Load data ===================================================================================================================
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<std::vector<float>> row;
            std::stringstream ss(line);
            float value;
            while (ss >> value) {
                row.push_back({ 0,0,value });
            }
            data.push_back({ row });
        }
        file.close();
    }
    else {
        throw std::runtime_error("Could not open file " + filePath);
    }

    // Remove rows with all zero values =============================================================================================
    for (auto it = data.begin(); it != data.end();) {
        bool allZero = true;
        for (const auto& val : *it) {
            if (val[2] != 0) {
                allZero = false;
                break;
            }
        }
        if (allZero) {
            it = data.erase(it);
        }
        else {
            ++it;
        }
    }
    // Remove columns with all zero values ===========================================================================================
    if (!data.empty()) {
        size_t numCols = data[0].size();

        for (size_t col = 0; col < numCols; ++col) {
            bool allZero = true;
            for (const auto& row : data) {
                if (row[col][2] != 0) {
                    allZero = false;
                    break;
                }
            }
            if (allZero) {
                for (auto& row : data) {
                    row.erase(row.begin() + col);
                }
                --col;
                --numCols;
            }
        }
    }
    // Fill in missing values by the next non-zero value =================================================================================
    for (auto& row : data) {
        for (size_t col = 0; col < row.size(); ++col) {
            if (row[col][2] == 0) {
                for (size_t k = col + 1; k < row.size(); ++k) {
                    if (row[k][2] != 0) {
                        row[col][2] = row[k][2];
                        break;
                    }
                }
            }
        }
    }

    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm
    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

	// Reshape data to 3D ===========================================================================================================
    std::vector<std::vector<std::vector<float>>> newData(num_rows, std::vector<std::vector<float>>(num_cols, std::vector<float>(3)));
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            newData[i][j][0] = j * x_spacing; // x
            newData[i][j][1] = i * y_spacing; // y
            newData[i][j][2] = data[i][j][2]; // z
        }
    }
    // vector [xrow][ycol][xyz]
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            std::array<float, 3> point = { newData[i][j][0], newData[i][j][1], newData[i][j][2] };
            ptCloud.push_back(point);
        }
    }
    // convert to normal pointclouds without any format
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            std::array<float, 3> point = { newData[i][j][0], newData[i][j][1], newData[i][j][2] };
            ptCloud.push_back(point);
        }
    }

}

void PclToMesh::createMesh() {
    auto& proMesh = nelems::mMesh::getInstance();
    size_t num_points = ptCloud.size(); // total number of points in the point cloud
    size_t num_rows =  100 ;               /* some appropriate calculation */ // You need to define how num_rows is computed
    size_t num_cols = num_points / num_rows; // Assuming a regular grid

    oMeshObject.changeName("pointcloud");
    oMeshObject.oMaterial.mColor = glm::vec3(0.8f, 0.7f, 0.75f);
    oMeshObject.ID = proMesh.getCurrentTimeMillis(0);

    // Add vertices from ptCloud
    for (auto& point : ptCloud) {
        nelems::VertexHolder vh;
        vh.mPos = { point[0], point[1], point[2] };
        oMeshObject.add_vertex(vh);
    }

    // Adding faces assuming a regular grid structure
    for (size_t i = 0; i < num_rows - 1; ++i) {
        for (size_t j = 0; j < num_cols - 1; ++j) {
            oMeshObject.add_vertex_index(i * num_cols + j);
            oMeshObject.add_vertex_index(i * num_cols + j + 1);
            oMeshObject.add_vertex_index((i + 1) * num_cols + j);

            oMeshObject.add_vertex_index(i * num_cols + j + 1);
            oMeshObject.add_vertex_index((i + 1) * num_cols + j + 1);
            oMeshObject.add_vertex_index((i + 1) * num_cols + j);
        }
    }
    oMeshObject.init();
}




// Create mesh
void PclToMesh::createMesh_pcl() {
    auto& proMesh = nelems::mMesh::getInstance();
    size_t num_points = ptCloud.size(); // Total number of points in the point cloud

    // ================================================
    // Convert std::vector<std::array<float, 3>> to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = num_points;
    cloud->height = 1; // Unorganized point cloud
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < ptCloud.size(); ++i) {
        cloud->points[i].x = ptCloud[i][0];
        cloud->points[i].y = ptCloud[i][1];
        cloud->points[i].z = ptCloud[i][2];
    }

    // ================================================
    // Using PCL to create a mesh
    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setKSearch(50); // Use 50 nearest neighbors to estimate the normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.compute(*normals);

    // Append the normals to the XYZ points
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Using PCL to create a mesh
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.1); // Set the maximum distance between connected points (maximum edge length)
    gp3.setMu(2.5); // Set typical value of the nearest neighbor distance
    gp3.setMaximumNearestNeighbors(100); // Set the maximum number of nearest neighbors that will be searched
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.reconstruct(triangles);

    // ================================================
    // Integrating the mesh data into oMeshObject
    oMeshObject.changeName("pointcloud");
    oMeshObject.oMaterial.mColor = glm::vec3(0.8f, 0.7f, 0.75f);
    oMeshObject.ID = proMesh.getCurrentTimeMillis(0);

    // Extract vertices
    for (const auto& vertex : cloud->points) {
        nelems::VertexHolder vh;
        vh.mPos = { vertex.x, vertex.y, vertex.z };
        oMeshObject.add_vertex(vh);
    }

    // Extract indices from the PCL mesh polygons
    for (const auto& polygon : triangles.polygons) {
        for (auto idx : polygon.vertices) {
            oMeshObject.add_vertex_index(idx);
        }
    }
    std::cout << "Number of polygons: " << triangles.polygons.size() << std::endl;
    for (const auto& polygon : triangles.polygons) {
        std::cout << "Polygon vertices: ";
        for (auto idx : polygon.vertices) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    }

    // Check if each polygon has exactly 3 vertices (for triangles)
    for (const auto& polygon : triangles.polygons) {
        if (polygon.vertices.size() != 3) {
            std::cout << "Found a polygon with vertex count != 3: " << polygon.vertices.size() << std::endl;
        }
    }

    oMeshObject.init();
}


void PclToMesh::processPointCloud() {
    //  Hung z
    /*loadPointCloud();
    filterZeroValues();
    createMesh();    */

    // Sang xyz
    loadDataCrXyz();
    createMesh();


    
}

void PclToMesh::addToMesh() {
    auto& proMesh = nelems::mMesh::getInstance();
    proMesh.pushback(oMeshObject);
}


void PclToMesh::loadDataCrXyz() {
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        std::vector<std::vector<float>> rows;
        while (std::getline(file, line)) {
            std::array<float,3> point;
            std::stringstream ss(line);
            ss >> point[0] >> point[1] >> point[2];
            ptCloud.push_back(point);
        }
        file.close();
    }
    else {
        throw std::runtime_error("Could not open file " + filePath);
    }
}
