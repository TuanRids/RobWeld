#ifndef CUDACAL_H
#define CUDACAL_H

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

typedef struct __pointkernel__ {
    float x, y, z;
};
typedef struct __LidarType__ {
    __pointkernel__* Ldata;
    int Lwidth;
    int Lheight;
    __device__ __pointkernel__& operator()(int row, int col) {
        return Ldata[row  * Lwidth + col];
    }
};
typedef struct __vertikernel__ {
    int v1, v2, v3;
};

extern "C" void launchKernel(float* d_data, __pointkernel__ * d_vertices, __vertikernel__ * d_triangles,
    size_t numRows, size_t numCols, float x_spacing, float y_spacing);

#endif

