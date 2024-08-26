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
void pushobjtomMesh(nelems::mMesh* proMesh, pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, pcl::PolygonMesh mesh, nui::StatusLogs* sttlogs, nui::SceneView* sceneview)
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
    oMeshObject.selected = true;
    oMeshObject.create_buffers();

    proMesh->getMesh()->push_back(std::make_shared<nelems::oMesh>(std::move(oMeshObject)));
    *sttlogs << "3D Object reconstructed: %i" + std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count();
    // rotation center
    sceneview->set_rotation_center();
    for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++) 
    {
            if (std::string(it->get()->oname).find( "ScanMesh") != std::string::npos) {it->get()->selected = true;}
    }
    // MeshObject.create_buffers();

}

PclToMesh::PclToMesh(): proMesh(nullptr), sttlogs(nullptr), robinit(nullptr), sceneview(nullptr)
{ 
    proMesh = &nelems::mMesh::getInstance(); 
    sttlogs = &nui::StatusLogs::getInstance(); 
    // get all files name "data_x......dat" in C:\\Robdat
    if (std::filesystem::exists("C:\\Robdat"))
    {
        for (const auto& entry : std::filesystem::directory_iterator("C:\\Robdat")) {
            if (entry.path().extension() == ".dat") {
                datpath = entry.path().string();
                break;
            }
        }
    }
}

void PclToMesh::processPointCloud() {
    

    
    
    // Check if the future is ready
    
    
 //   auto resulttest = Create3DPCL(data);
 //   auto vertices = resulttest.first;
	//auto mesh = resulttest.second;

 //   if (vertices && mesh.polygons.size() > 0) {
 //       pushobjtomMesh(proMesh, vertices, mesh, sttlogs);
 //   }
 //   // reset future
 //   future = nullptr;
}

void PclToMesh::show3d_data()
{
    static std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>> future;

    // Check for new .dat file
    if (!std::filesystem::exists("C:\\Robdat")) { return; }
    std::string new_dat_file;
    for (const auto& entry : std::filesystem::directory_iterator("C:\\Robdat")) {
        if (entry.path().extension() == ".dat") {
            new_dat_file = entry.path().string();
            break;
        }
    }

    // If there is a new data file, process it
    if (new_dat_file != datpath) {
        datpath = new_dat_file;
        std::ifstream infile(datpath, std::ios::in);
        data.clear();
        std::string line;

        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            std::vector<float> row;
            float f;
            while (iss >> f) {
                row.push_back(f);
            }
            data.push_back(row);
        }
        
        future = std::async(std::launch::async, [this]() { return Create3DPCL(data); });
        *sttlogs << "Started processing 3D Object: "
            + datpath.substr(datpath.find_last_of("\\/") + 1, datpath.find_last_of(".") - datpath.find_last_of("\\/") - 1);
    }
    if (!sceneview){    sceneview = &nui::SceneView::getInstance();}
    // Analyze the future
    if (future.valid() && future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto result = future.get();
        auto vertices = result.first;
        auto mesh = result.second;

        if (vertices && !mesh.polygons.empty()) {
            pushobjtomMesh(proMesh, vertices, mesh, sttlogs, sceneview);
        }

        // Reset future
        future = std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>>();
    }
}




