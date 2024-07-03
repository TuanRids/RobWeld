#include "pch.h"
#include "pclToMesh.h"


#include <Eigen/Dense>


#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <glm/glm.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h> // Use the OpenMP version for parallel processing
#include <pcl/filters/voxel_grid.h> // For downsampling
#include <pcl/kdtree/kdtree_flann.h> // For nearest neighbors
#include <pcl/surface/poisson.h> // Alternatively, using Poisson surface reconstruction

#include "omp.h"


void PclToMesh::Create3DPCL() {
    auto start = std::chrono::high_resolution_clock::now();
    auto logTime = [&](const std::string& message) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
        std::string content = message + " [" + std::to_string(duration) + "ms]";
        *sttlogs << content;
        start = std::chrono::high_resolution_clock::now();
        };

    std::vector<std::vector<float>> data;
    nelems::oMesh oMeshObject;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<float> row;
            std::stringstream ss(line);
            float value;
            while (ss >> value) {
                row.push_back(value);
            }
            data.push_back(row);
        }
        file.close();
    }
    else {
        *sttlogs << "[ERROR] Could not open file!";
        return;
    }

    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_size = 100.0f, y_size = 200.0f;
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

    cloud->width = num_cols;
    cloud->height = num_rows;
    cloud->is_dense = false;
    cloud->points.resize(num_cols * num_rows);

    logTime("Data loaded");

#pragma omp parallel for
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            cloud->points[i * num_cols + j] = pcl::PointXYZ(j * x_spacing, i * y_spacing, data[i][j]);
        }
    }

    logTime("Data converted");

    if (cloud->empty()) {
        logTime( "[ERROR] Input cloud is empty after loading data!");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(1.0f, 1.0f, 1.0f);
    voxelGrid.filter(*cloudFiltered);

    if (cloudFiltered->empty()) {
        logTime( "[ERROR] Filtered cloud is empty after downsampling!");
        return;
    }

    logTime("Data downsampled");

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normalEstimation.setInputCloud(cloudFiltered);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setKSearch(100);
    normalEstimation.compute(*normals);

    if (normals->empty()) {
        logTime( "[ERROR] Normals cloud is empty after normal estimation!");
        return;
    }

    logTime("Normals estimated");

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloudFiltered, *normals, *cloudWithNormals);

    if (cloudWithNormals->empty()) {
        logTime("[ERROR] Cloud with normals is empty after concatenation!");
        return;
    }

    logTime("Data concatenated");

    pcl::search::KdTree<pcl::PointNormal>::Ptr treeWithNormals(new pcl::search::KdTree<pcl::PointNormal>());
    treeWithNormals->setInputCloud(cloudWithNormals);

    if (!treeWithNormals->getInputCloud()) {
        logTime("[ERROR] Failed to create KD-Tree with normals!");
        return;
    }

    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(8);
    poisson.setInputCloud(cloudWithNormals);
    poisson.reconstruct(*triangles);

    logTime("Poisson reconstruction");

    oMeshObject.changeName("Poisson_Mesh");
    oMeshObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(triangles->cloud, *vertices);

    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        oMeshObject.add_vertex(vh);
    }

    oMeshObject.oMaterial.position = glm::vec3(oMeshObject.mVertices[0].mPos[0], oMeshObject.mVertices[0].mPos[1], oMeshObject.mVertices[0].mPos[2]);

    for (const auto& polygon : triangles->polygons) {
        for (const auto& index : polygon.vertices) {
            oMeshObject.add_vertex_index(index);
        }
    }
    oMeshObject.calculate_normals();
    oMeshObject.init();
    proMesh->pushback(oMeshObject);

    logTime("Mesh conversion completed");
}


void PclToMesh::processPointCloud() {
    Create3DPCL(); 
}


