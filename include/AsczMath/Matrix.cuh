#ifndef MATRIX_CUH
#define MATRIX_CUH

#include <Vector.cuh>

struct Mat4 {
    float data[4][4] = {0};
    __host__ __device__ Mat4();
    __host__ __device__ Mat4(float data[4][4]);

    // Basic operations
    __host__ __device__ Mat4 operator+(const Mat4 &mat);
    __host__ __device__ Mat4 operator-(const Mat4 &mat);
    __host__ __device__ Mat4 operator*(const float scl);

    // Advanced operations
    __host__ __device__ Flt4 operator*(const Flt4 &vec);
    __host__ __device__ Mat4 operator*(const Mat4 &mat);
    __host__ __device__ float det(); // Determinant
};

#endif