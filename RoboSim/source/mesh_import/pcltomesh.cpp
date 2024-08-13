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
#include "future"


void PclToMesh::logTime(std::chrono::time_point<std::chrono::high_resolution_clock>& start, const std::string& message) {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
    std::string content = message + " [" + std::to_string(duration) + "ms]";
    *sttlogs << content;
    start = std::chrono::high_resolution_clock::now();
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh> Create3DPCL(std::vector<std::vector<float>> data) {
    auto start = std::chrono::high_resolution_clock::now();

    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_size = 100.0f, y_size = 200.0f;
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::Vertices> triangles;

    for (int i = 0; i < num_rows - 1; ++i) {
        for (int j = 0; j < num_cols - 1; ++j) {
            int index = i * num_cols + j;

            pcl::PointXYZ p1(j * x_spacing, i * y_spacing, data[i][j]);
            pcl::PointXYZ p2((j + 1) * x_spacing, i * y_spacing, data[i][j + 1]);
            pcl::PointXYZ p3(j * x_spacing, (i + 1) * y_spacing, data[i + 1][j]);
            pcl::PointXYZ p4((j + 1) * x_spacing, (i + 1) * y_spacing, data[i + 1][j + 1]);

            vertices->points.push_back(p1);
            vertices->points.push_back(p2);
            vertices->points.push_back(p3);
            vertices->points.push_back(p4);

            pcl::Vertices v1, v2;
            v1.vertices = { static_cast<int>(index * 4), static_cast<int>(index * 4 + 1), static_cast<int>(index * 4 + 2) };
            v2.vertices = { static_cast<int>(index * 4 + 1), static_cast<int>(index * 4 + 3), static_cast<int>(index * 4 + 2) };

            triangles.push_back(v1);
            triangles.push_back(v2);
        }
    }

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*vertices, mesh.cloud);
    mesh.polygons = triangles;
    return std::make_pair(vertices, mesh);
}
void pushobjtomMesh(nelems::mMesh* proMesh, pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, pcl::PolygonMesh mesh, nui::StatusLogs* sttlogs)
{
    auto start = std::chrono::high_resolution_clock::now();
    proMesh->delete_byname("ScanMesh");
    nelems::oMesh oMeshObject;
    oMeshObject.changeName("ScanMesh");
    oMeshObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    oMeshObject.oMaterial.mMetallic = 0.99;
    oMeshObject.oMaterial.mRoughness = 0.2;
    
    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        oMeshObject.add_vertex(vh);
    }

    for (const auto& polygon : mesh.polygons) {
        for (const auto& index : polygon.vertices) {
            oMeshObject.add_vertex_index(index);
        }
    }

    glm::vec3 center = glm::vec3(0.0f);
    for (const auto& vertex : oMeshObject.mVertices) {
        center += vertex.mPos;
    }
    center /= static_cast<float>(oMeshObject.mVertices.size());
    oMeshObject.oMaterial.position = center;

    oMeshObject.ID = proMesh->getCurrentTimeMillis(0);
    oMeshObject.calculate_normals();
    oMeshObject.init();
    oMeshObject.move(500 - center.x, 0 - center.y, -300 - center.z);
    oMeshObject.rotate(0, 180, 0);
    oMeshObject.create_buffers();
    proMesh->getMesh()->push_back(std::make_shared<nelems::oMesh>(std::move(oMeshObject)));
    *sttlogs << "3D Object reconstructed: %i" + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count();
}

void PclToMesh::processPointCloud(int keyID) {
    static std::unique_ptr<std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>>> future = nullptr;

    if (data.size() == 0 || data.size() > 10000) {
        return;
    }

    // Check if key has changed or if no async task is running
    if (keyID != key) {
        future =std::make_unique<std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>>>();
        *future = std::async(std::launch::async, [this]() { return Create3DPCL(data); });
        key = keyID;
        *sttlogs << "Started processing 3D Object: Key " + std::to_string(keyID);
    }
    // Check if the future is ready
    if (future && future->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto result = future->get();
        auto vertices = result.first;
        auto mesh = result.second;

        if (vertices && mesh.polygons.size() > 0) {
            pushobjtomMesh(proMesh, vertices, mesh,sttlogs);
        }
        // reset future
		future = nullptr;
    }
}



