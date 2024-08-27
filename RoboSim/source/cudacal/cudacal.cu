#include "cudacal.h"

__global__ void calculateVerticesAndTriangles(__pointkernel__* d_vertices, __vertikernel__* d_triangles, float* d_data,
    size_t numRows, size_t numCols, float x_spacing, float y_spacing) {
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;

    if (row < numRows && col < numCols) {
        int idx = row * numCols + col;

        // Calculate vertex position
        d_vertices[idx].x = col * x_spacing;
        d_vertices[idx].y = row * y_spacing;
        d_vertices[idx].z = d_data[idx];

        // Calculate triangles
        if (row < numRows - 1 && col < numCols - 1) {
            int top_left = idx;
            int top_right = idx + 1;
            int bottom_left = idx + numCols;
            int bottom_right = bottom_left + 1;

            d_triangles[2 * idx].v1 = top_left;
            d_triangles[2 * idx].v2 = top_right;
            d_triangles[2 * idx].v3 = bottom_left;

            d_triangles[2 * idx + 1].v1 = top_right;
            d_triangles[2 * idx + 1].v2 = bottom_right;
            d_triangles[2 * idx + 1].v3 = bottom_left;
        }
    }
}

extern "C" void launchKernel(float* d_data, __pointkernel__ * d_vertices, __vertikernel__ * d_triangles,
    size_t numRows, size_t numCols, float x_spacing, float y_spacing) {
    dim3 blockSize(16, 16);
    dim3 gridSize((numCols + blockSize.x - 1) / blockSize.x, (numRows + blockSize.y - 1) / blockSize.y);

    calculateVerticesAndTriangles << <gridSize, blockSize >> > (d_vertices, d_triangles, d_data, numRows, numCols, x_spacing, y_spacing);

    cudaDeviceSynchronize();
}
