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
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh> Create3DPCL_CUDA(std::vector<std::vector<float>> data) {
    size_t numRows = data.size();
    size_t numCols = data[0].size();
    float x_size = 100.0f, y_size = 200.0f;
    float x_spacing = x_size / (numCols - 1);
    float y_spacing = y_size / (numRows - 1);

    size_t numVertices = numRows * numCols;
    size_t numTriangles = (numRows - 1) * (numCols - 1) * 2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    vertices->points.resize(numVertices);
    std::vector<pcl::Vertices> triangles(numTriangles);

    // Allocate memory on the GPU
    float* d_data = nullptr;
    __pointkernel__* d_vertices = nullptr;
    __vertikernel__* d_triangles = nullptr;

    cudaMalloc((void**)&d_vertices, numVertices * sizeof(__pointkernel__));
    cudaMalloc((void**)&d_triangles, numTriangles * sizeof(__vertikernel__));
    cudaMalloc((void**)&d_data, numRows * numCols * sizeof(float));
    cudaMemcpy(d_data, &data[0][0], numRows * numCols * sizeof(float), cudaMemcpyHostToDevice);
    std::cout << ", d_triangles: " << d_triangles << std::endl;

    // Launch kernel to calculate vertices and triangles
    launchKernel(d_data, d_vertices, d_triangles, numRows, numCols, x_spacing, y_spacing);

    // Copy results back to the CPU
    cudaMemcpy(vertices->points.data(), d_vertices, numVertices * sizeof(__pointkernel__), cudaMemcpyDeviceToHost);
    cudaMemcpy(triangles.data(), d_triangles, numTriangles * sizeof(__vertikernel__), cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(d_data);
    cudaFree(d_vertices);
    cudaFree(d_triangles);

    // Prepare the mesh
    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*vertices, mesh.cloud);
    mesh.polygons = triangles;
    return std::make_pair(vertices, mesh);
}


std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh> Create3DPCL(std::vector<std::vector<float>> data) {
    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_size = 100.0f, y_size = 200.0f;
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr bvertices(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::Vertices> triangles;

    bvertices->points.resize(num_rows * num_cols);

#pragma omp parallel for
    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            bvertices->points[i * num_cols + j] = pcl::PointXYZ(j * x_spacing, i * y_spacing, data[i][j]);

            if (i < num_rows - 1 && j < num_cols - 1) {
                pcl::Vertices v1, v2;
                int top_left = i * num_cols + j;
                int top_right = top_left + 1;
                int bottom_left = (i + 1) * num_cols + j;
                int bottom_right = bottom_left + 1;

                v1.vertices = { top_left, top_right, bottom_left };
                v2.vertices = { top_right, bottom_right, bottom_left };
                #pragma omp critical
                {
                    triangles.push_back(v1);
                    triangles.push_back(v2);
                }
            }
        }
    }

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*bvertices, mesh.cloud);
    mesh.polygons = triangles;
    return std::make_pair(bvertices, mesh);
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
    glm::vec3 center = glm::vec3(0.0f);

#pragma omp parallel for reduction(+:center)
    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        oMeshObject.add_vertex(vh);
        center += vh.mPos;
    }

#pragma omp parallel for
    for (const auto& polygon : mesh.polygons) {
        for (const auto& index : polygon.vertices) {
            oMeshObject.add_vertex_index(index);
        }
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
    // rotation center
    sceneview->set_rotation_center();
    for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
    {
        if (std::string(it->get()->oname).find("ScanMesh") != std::string::npos) { it->get()->selected = false; }
    }
}

int extractNumberFromFilename(const std::string& filename) {
    size_t startPos = filename.find("__") + 2;
    size_t endPos = filename.find("__", startPos);
    return std::stoi(filename.substr(startPos, endPos - startPos));
}

PclToMesh::PclToMesh(): proMesh(nullptr), sttlogs(nullptr), robinit(nullptr), sceneview(nullptr)
{ 
    proMesh = &nelems::mMesh::getInstance(); 
    sttlogs = &nui::StatusLogs::getInstance(); 
    // get all files name "data_x......dat" in C:\\Robdat
    if (!std::filesystem::exists("C:\\Robdat"))
    {
        // create folder
		std::filesystem::create_directory("C:\\Robdat");
    }
    for (const auto& entry : std::filesystem::directory_iterator("C:\\Robdat")) {
        if (entry.path().extension() == ".dat") {
            datpath = entry.path().string();
            break;
        }
    }
}

// fread method for reading binary files
void readFixedFormatFile(const std::string& filename, std::vector<std::vector<float>>& data) {
    data.clear();
    std::ifstream infile(filename, std::ios::binary | std::ios::ate);

    // get __int number__ from filename

    if (!infile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::streamsize fileSize = infile.tellg();
    infile.seekg(0, std::ios::beg);

    size_t numCols = 1024; 
    size_t numRows = fileSize / (numCols * sizeof(float)); 

    std::vector<float> buffer(numRows * numCols);

    if (numRows < extractNumberFromFilename(filename) - 6 ) { return; }

    if (infile.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        data.resize(numRows, std::vector<float>(numCols));

        for (size_t i = 0; i < numRows; ++i) {
            std::copy(buffer.begin() + i * numCols, buffer.begin() + (i + 1) * numCols, data[i].begin());
        }
    }
    else {
        std::cerr << "Error reading file: " << filename << std::endl;
    }

    return;
}


void PclToMesh::show3d_data()
{
    static std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>> future;
    static auto lstart = std::chrono::high_resolution_clock::now();

    static bool useCUDA = ([this] {
        int deviceCount = 0;
        bool cudaAvailable = (cudaGetDeviceCount(&deviceCount) == cudaSuccess) && (deviceCount > 0);
        *sttlogs << (cudaAvailable ? "***** CUDA IS AVAILABLE TO USE *****" : "***** CUDA is NOT AVAILABLE *****");
        return cudaAvailable;
        })();

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
        lstart = std::chrono::high_resolution_clock::now();
        *sttlogs << "Started processing 3D Object: "
            + datpath.substr(datpath.find_last_of("\\/") + 1, datpath.find_last_of(".") - datpath.find_last_of("\\/") - 1);
        datpath = new_dat_file;
        std::ifstream infile(datpath, std::ios::in);        
        std::string line;     

        readFixedFormatFile(datpath,data);        
        if (data.size() < 5) { datpath = ""; return; }
        
        future = std::async(std::launch::async, [this]() { return Create3DPCL(data); }); // 0.45s
        //}
        *sttlogs << "Analyzing for 3D Object---";
    }
    if (!sceneview){    sceneview = &nui::SceneView::getInstance();}


    // Analyze the future
    if (future.valid() && future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh> result = future.get();
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices = result.first;
        pcl::PolygonMesh mesh = result.second;

        if (vertices && !mesh.polygons.empty()) {
            pushobjtomMesh(proMesh, vertices, mesh, sttlogs, sceneview);
        }
        // Reset future
        future = std::future<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PolygonMesh>>();
        auto tempt1 = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lstart).count());
        *sttlogs << "3D object is created in:" + tempt1 + " ms.";

    }
}




