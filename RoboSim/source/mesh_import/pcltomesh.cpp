#define _CRT_SECURE_NO_WARNINGS
#include "pch.h"
#include "pclToMesh.h"
#include <fstream>
#include <sstream>
#include <map>
#include <omp.h>

#include <Eigen/Dense>


#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <glm/glm.hpp>
#include <vector>
#include <string>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h> // Use the OpenMP version for parallel processing
#include <pcl/filters/voxel_grid.h> // For downsampling
#include <pcl/kdtree/kdtree_flann.h> // For nearest neighbors
#include <pcl/surface/poisson.h> // Alternatively, using Poisson surface reconstruction

// segmentation
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <cmath>
#include "elems/vertex_holder.h"

#include "chrono"





void PclToMesh::filterZeroValues() {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<std::vector<float>>> data; // data format [rows][cols][3]
    // Load data: ONLY have Z values
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
    // Remove rows with all zero values 
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
    std::cout << "remzero" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Remove columns with all zero values 
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
    std::cout << "Remove col" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Fill in missing values by the next non-zero value 
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
    std::cout << "fillzero" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm
    size_t num_rows = data.size();
    size_t num_cols = data[0].size();
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

	// Reshape data to 3D 
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
    std::cout <<"end" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
}

/* void PclToMesh::filterZeroValues() {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> data; // data format [rows][cols]

    // Load data: ONLY have Z values
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
        throw std::runtime_error("Could not open file " + filePath);
    }

    std::cout << "Time load " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;

    // Convert to Eigen matrix
    Eigen::MatrixXf mat(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[i].size(); ++j) {
            mat(i, j) = data[i][j];
        }
    }

    // Remove rows with all zero values
    Eigen::MatrixXf nonZeroRows = mat.rowwise().any() ? mat : Eigen::MatrixXf();
    mat = nonZeroRows;

    std::cout << "Time remove zero rows " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;

    // Remove columns with all zero values
    Eigen::MatrixXf nonZeroCols = mat.colwise().any() ? mat : Eigen::MatrixXf();
    mat = nonZeroCols;

    std::cout << "Time remove zero cols " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;

    // Fill missing values with the next non-zero value
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            if (mat(i, j) == 0) {
                for (int k = j + 1; k < mat.cols(); ++k) {
                    if (mat(i, k) != 0) {
                        mat(i, j) = mat(i, k);
                        break;
                    }
                }
            }
        }
    }

    std::cout << "Time fill zero values " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;

    float x_size = 100.0f; // mm
    float y_size = 200.0f; // mm
    size_t num_rows = mat.rows();
    size_t num_cols = mat.cols();
    float x_spacing = x_size / (num_cols - 1);
    float y_spacing = y_size / (num_rows - 1);

    // Prepare point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = num_cols;
    cloud->height = num_rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            float x = j * x_spacing;
            float y = i * y_spacing;
            float z = mat(i, j);
            cloud->points[i * num_cols + j] = pcl::PointXYZ(x, y, z);
        }
    }

    std::cout << "Time to create point cloud " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;

    ptCloud = cloud;

    std::cout << "End " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
}*/



void PclToMesh::createMesh() {
    auto& proMesh = nelems::mMesh::getInstance();
    size_t num_points = ptCloud.size(); // total number of points in the point cloud
    std::shared_ptr<size_t> numsize = std::make_shared<size_t>(ptCloud.size());    

    std::set<float> uniqueY;
    for (const auto& point : ptCloud) {    uniqueY.insert(point[1]);    }
    num_rows = uniqueY.size();
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
    oMeshObject.oMaterial.position = glm::vec3(ptCloud[0][0], ptCloud[0][1], ptCloud[0][2]);
    oMeshObject.calculate_normals();
    oMeshObject.init();
}
// =======================================================================================================
// =======================================================================================================
// TODO Update pcl algorithm



void PclToMesh::segmentPointCloud() {

    // Remake low quality object
    // Load point cloud data into PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : ptCloud) {
        cloud->points.emplace_back(point[0], point[1], point[2]);
    }
    cloud->width = ptCloud.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // Downsample the point cloud to speed up processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);             // ************ ************ 1 reduce for slower but more accurate
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust the leaf size to be more aggressive 0.05 & 0.001
    voxelGrid.filter(*cloudFiltered);

    // Estimate normals using the OpenMP version for parallel processing
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normalEstimation.setInputCloud(cloudFiltered);
    normalEstimation.setSearchMethod(tree); // ************ ************ increase setKSearch for more accurate but slower
    normalEstimation.setKSearch(10); // Reduced number of neighbors for faster computation 40 50
    normalEstimation.compute(*normals);

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloudFiltered, *normals, *cloudWithNormals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr treeWithNormals(new pcl::search::KdTree<pcl::PointNormal>());
    treeWithNormals->setInputCloud(cloudWithNormals);

    // Manually create the mesh
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    pcl::Poisson<pcl::PointNormal> poisson;  // ************ ************ increase depth for more accurate but slower
    poisson.setDepth(8); // Set the depth of the tree used for reconstruction 8 10
    poisson.setInputCloud(cloudWithNormals);
    poisson.reconstruct(*triangles);

    nelems::oMesh tempObject;

    // Convert PCL mesh to oMesh format
    tempObject.changeName("Poisson_Mesh");
    tempObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    tempObject.oMaterial.position = glm::vec3(ptCloud[0][0], ptCloud[0][1], ptCloud[0][2]);

    // Add vertices from PCL mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(triangles->cloud, *vertices);

    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        tempObject.add_vertex(vh);
    }

    // Add vertex indices from PCL mesh
    for (const auto& polygon : triangles->polygons) {
        for (const auto& index : polygon.vertices) {
            tempObject.add_vertex_index(index);
        }
    }

    // Parameters
    std::vector<nelems::VertexHolder>& Segvertices = tempObject.mVertices;

    float neigh_radius = 0.02f;
    std::vector<float> curvature(Segvertices.size(), 0.0f);

    #pragma omp parallel for schedule(dynamic)
#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < Segvertices.size(); ++i) {
        std::vector<glm::vec3> local_points;
        for (size_t j = 0; j < Segvertices.size(); ++j) {
            if (i != j && distance(Segvertices[i].mPos, Segvertices[j].mPos) < neigh_radius) {
                local_points.push_back(Segvertices[j].mPos);
            }
        }

        if (local_points.size() > 1) {
            glm::vec3 centroid(0.0f);
            for (const auto& point : local_points) {
                centroid += point;
            }
            centroid /= static_cast<float>(local_points.size());

            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (const auto& point : local_points) {
                Eigen::Vector3f vec = Eigen::Vector3f(point.x, point.y, point.z) - Eigen::Vector3f(centroid.x, centroid.y, centroid.z);
                covariance += vec * vec.transpose();
            }
            covariance /= static_cast<float>(local_points.size());

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
            Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();

            curvature[i] = eigen_values.minCoeff();
        }
    }


    segmentedObject.changeName("Segmented");
    segmentedObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    segmentedObject.oMaterial.position = glm::vec3(ptCloud[0][0], ptCloud[0][1], ptCloud[0][2]);

    float max_curvature = *std::max_element(curvature.begin(), curvature.end());
    for (auto& curv : curvature) {
        curv /= max_curvature;
    }


    // Create mesh for segmented object
    for (size_t i = 0; i < Segvertices.size(); ++i) {
        if (curvature[i] > 0.5f) { 
            segmentedObject.add_vertex(Segvertices[i]);
            segmentedObject.add_vertex_index(static_cast<unsigned int>(i));
        }
    }
    segmentedObject.calculate_normals();
    segmentedObject.init();
    // Push the segmented object to the global mesh instance
    auto& proMesh = nelems::mMesh::getInstance();
    proMesh.pushback(segmentedObject);
}


#include <vector>

void exportToOBJ(const nelems::oMesh& oMeshObject, const std::string& outputFilename) {
    aiScene scene;
    scene.mRootNode = new aiNode();
    scene.mMaterials = new aiMaterial * [1];
    scene.mMaterials[0] = new aiMaterial();
    scene.mNumMaterials = 1;

    scene.mMeshes = new aiMesh * [1];
    scene.mMeshes[0] = new aiMesh();
    scene.mNumMeshes = 1;
    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mMeshes[0] = 0;
    scene.mRootNode->mNumMeshes = 1;

    aiMesh* mesh = scene.mMeshes[0];
    mesh->mMaterialIndex = 0;

    // Fill vertices
    mesh->mNumVertices = oMeshObject.mVertices.size();
    std::vector<aiVector3D> vertices(mesh->mNumVertices);
    std::vector<aiVector3D> normals(mesh->mNumVertices);

    for (size_t i = 0; i < oMeshObject.mVertices.size(); ++i) {
        vertices[i] = aiVector3D(oMeshObject.mVertices[i].mPos.x, oMeshObject.mVertices[i].mPos.y, oMeshObject.mVertices[i].mPos.z);
        normals[i] = aiVector3D(oMeshObject.mVertices[i].mNormal.x, oMeshObject.mVertices[i].mNormal.y, oMeshObject.mVertices[i].mNormal.z);
    }

    mesh->mVertices = vertices.data();
    mesh->mNormals = normals.data();

    // Fill faces
    mesh->mNumFaces = oMeshObject.mVertexIndices.size() / 3;
    std::vector<aiFace> faces(mesh->mNumFaces);
    std::vector<std::vector<unsigned int>> allIndices(mesh->mNumFaces, std::vector<unsigned int>(3));
    for (size_t i = 0; i < mesh->mNumFaces; ++i) {
        faces[i].mNumIndices = 3;
        allIndices[i][0] = oMeshObject.mVertexIndices[i * 3];
        allIndices[i][1] = oMeshObject.mVertexIndices[i * 3 + 1];
        allIndices[i][2] = oMeshObject.mVertexIndices[i * 3 + 2];
        faces[i].mIndices = allIndices[i].data();
    }

    mesh->mFaces = faces.data();

    Assimp::Exporter exporter;
    if (exporter.Export(&scene, "obj", outputFilename) != AI_SUCCESS) {
        std::cerr << "Failed to export mesh: " << exporter.GetErrorString() << std::endl;
    }
    else {
        std::cout << "Mesh successfully exported to " << outputFilename << std::endl;
    }
    try
    {
        // Clean up
        delete scene.mRootNode;
        for (size_t i = 0; i < scene.mNumMaterials; ++i) {
            delete scene.mMaterials[i];
        }
        delete[] scene.mMaterials;
        for (size_t i = 0; i < scene.mNumMeshes; ++i) {
            delete scene.mMeshes[i];
        }
        delete[] scene.mMeshes;
    }
    catch (const std::exception& e)
	{
		std::cerr << "Failed to clean up: " << e.what() << std::endl;
	}
}


void PclToMesh::createMesh_pcl() {
    auto start = std::chrono::high_resolution_clock::now();
    // Load point cloud data into PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : ptCloud) {
        cloud->points.emplace_back(point[0], point[1], point[2]);
    }
    cloud->width = ptCloud.size();
    cloud->height = 1;
    cloud->is_dense = false;
    std::cout << "Time load " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Downsample the point cloud to speed up processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);             // ************ ************ 1 reduce for slower but more accurate
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust the leaf size to be more aggressive 0.05 & 0.001
    std::cout << "Time downsample " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Estimate normals using the OpenMP version for parallel processing
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normalEstimation.setInputCloud(cloudFiltered);
    normalEstimation.setSearchMethod(tree); // ************ ************ increase setKSearch for more accurate but slower
    normalEstimation.setKSearch(20); // Reduced number of neighbors for faster computation 40 50
    normalEstimation.compute(*normals);
    std::cout << "Time estimate Normals " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloudFiltered, *normals, *cloudWithNormals);
    std::cout << "Time concatenate " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr treeWithNormals(new pcl::search::KdTree<pcl::PointNormal>());
    treeWithNormals->setInputCloud(cloudWithNormals);
    std::cout << "Time create search tree " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Manually create the mesh
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
    pcl::Poisson<pcl::PointNormal> poisson;  // ************ ************ increase depth for more accurate but slower
    poisson.setDepth(10); // Set the depth of the tree used for reconstruction 8 10
    poisson.setInputCloud(cloudWithNormals);
    poisson.reconstruct(*triangles);
    std::cout << "Time poisson " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms" << std::endl;
    // Convert PCL mesh to oMesh format
    oMeshObject.changeName("Poisson_Mesh");
    oMeshObject.oMaterial.mColor = glm::vec3(0.2f, 0.4f, 0.75f);
    oMeshObject.oMaterial.position = glm::vec3(ptCloud[0][0], ptCloud[0][1], ptCloud[0][2]);

    // Add vertices from PCL mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(triangles->cloud, *vertices);

    for (const auto& point : vertices->points) {
        nelems::VertexHolder vh;
        vh.mPos = { point.x, point.y, point.z };
        oMeshObject.add_vertex(vh);
    }

    // Add vertex indices from PCL mesh
    for (const auto& polygon : triangles->polygons) {
        for (const auto& index : polygon.vertices) {
            oMeshObject.add_vertex_index(index);
        }
    }
    oMeshObject.calculate_normals();
    oMeshObject.init();

    // export OBJ.
    // exportToOBJ(oMeshObject, "check_pcl//output.obj");

}


// =======================================================================================================
// =======================================================================================================




void PclToMesh::processPointCloud() {
    //  Hung z
    filterZeroValues();
    //createMesh();    

    // Sang xyz
    //loadDataCrXyz();
    createMesh_pcl();
    //segmentPointCloud();
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


