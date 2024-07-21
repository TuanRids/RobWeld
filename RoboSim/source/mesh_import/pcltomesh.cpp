#define _CRT_SECURE_NO_WARNINGS
#include "pch.h"
#include "pclToMesh.h"

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


void PclToMesh::Create3DPCL(const float& SizeLeaf, const unsigned int& poidepth) {
    auto start = std::chrono::high_resolution_clock::now();
    auto logTime = [&](const std::string& message) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
        std::string content = message + " [" + std::to_string(duration) + "ms]";
        *sttlogs << content;
        start = std::chrono::high_resolution_clock::now();
        };

    nelems::oMesh oMeshObject;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_size = 100.0f, y_size = 200.0f;
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

    cloud->width = num_cols;
    cloud->height = num_rows;
    cloud->is_dense = false;
    cloud->points.resize(num_cols * num_rows);

#pragma omp parallel for
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            cloud->points[i * num_cols + j] = pcl::PointXYZ(j * x_spacing, i * y_spacing, data[i][j]);
        }
    }

    logTime("Data loaded & converted");

    if (cloud->empty()) {
        logTime( "[ERROR] Input cloud is empty after loading data!");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(SizeLeaf, SizeLeaf, SizeLeaf);
    voxelGrid.filter(*cloudFiltered);

    if (cloudFiltered->empty()) {
        logTime( "[ERROR] Filtered cloud is empty after downsampling!");
        return;
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normalEstimation.setInputCloud(cloudFiltered);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setKSearch(10);
    normalEstimation.compute(*normals);

    if (normals->empty()) {
        logTime( "[ERROR] Normals cloud is empty after normal estimation!");
        return;
    }

    logTime("downsampled & Normals estimated");

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloudFiltered, *normals, *cloudWithNormals);

    if (cloudWithNormals->empty()) {
        logTime("[ERROR] Cloud with normals is empty after concatenation!");
        return;
    }

    logTime("Concatenated");

    pcl::search::KdTree<pcl::PointNormal>::Ptr treeWithNormals(new pcl::search::KdTree<pcl::PointNormal>());
    treeWithNormals->setInputCloud(cloudWithNormals);

    if (!treeWithNormals->getInputCloud()) {
        logTime("[ERROR] Failed to create KD-Tree with normals!");
        return;
    }

    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(poidepth);   // Depth field
    poisson.setIsoDivide(8);    // Number of iso divisions
	poisson.setManifold(true);  // Manifold mesh
	poisson.setSamplesPerNode(5); // Number of samples per node
	poisson.setPointWeight(4.0f); // Point weight
	poisson.setSearchMethod(treeWithNormals);
    poisson.setInputCloud(cloudWithNormals);
    poisson.reconstruct(*triangles);

    logTime("Poisson reconstruction");
    proMesh->delete_byname("Poisson_Mesh");

    oMeshObject.changeName("Poisson_Mesh");
    oMeshObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(triangles->cloud, *vertices);

    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        oMeshObject.add_vertex(vh);
    }

    for (const auto& polygon : triangles->polygons) {for (const auto& index : polygon.vertices) {oMeshObject.add_vertex_index(index);}}

    glm::vec3 center = glm::vec3(0.0f);
    for (const auto& vertex : oMeshObject.mVertices) {center += vertex.mPos;    }
    center /= static_cast<float>(oMeshObject.mVertices.size());
    oMeshObject.oMaterial.position = center;

    oMeshObject.ID = proMesh->getCurrentTimeMillis(0);
    oMeshObject.calculate_normals();
    oMeshObject.init();
    oMeshObject.move(500-center.x, 0-center.y, -300-center.z);
    oMeshObject.create_buffers();
    proMesh->getMesh()->push_back(std::make_shared<nelems::oMesh>(std::move(oMeshObject)));
    logTime("Mesh conversion completed");
}


void PclToMesh::processPointCloud() {
    if (!robinit) { robinit = &RobInitFile::getinstance(); }
    try {
        std::string temptvalue;
        robinit->get_settings("creating_speed", temptvalue);
        creating_speed = std::stoi(temptvalue);
    }catch(...){}
    if (creating_speed == 0) { Create3DPCL(0.5, 10); }
    else if (creating_speed == 1) { Create3DPCL(0.5, 12); }
	else if (creating_speed == 2) { Create3DPCL(0.05, 12); }
    // Normal: 0.5, 12           6s
	// Fastest: 0.5, 10        1.5s
	// Detailest: 0.05, 12      20s
}


