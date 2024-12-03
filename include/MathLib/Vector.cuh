#ifndef VECTOR_CUH
#define VECTOR_CUH

#include <iostream>
#include <vector>
#include <cstdio>
#include <cmath>

#define UInt unsigned int
#define ULInt unsigned long int
#define ULLInt unsigned long long int
#define LLInt long long int

#define Vecs2f std::vector<Vec2f>
#define Vecs3f std::vector<Vec3f>
#define Vecs4f std::vector<Vec4f>

#define M_PI 3.14159265358979323846 // 180 degrees
#define M_PI_2 1.57079632679489661923 // 90 degrees
#define M_2_PI 6.28318530717958647692 // 360 degrees

struct Vec2f {
    float x, y;
    __host__ __device__ Vec2f();
    __host__ __device__ Vec2f(float x, float y);

    // Basic operations
    __host__ __device__ Vec2f operator+(const Vec2f &vec);
    __host__ __device__ Vec2f operator+(const float t);
    __host__ __device__ Vec2f operator-(const Vec2f &vec);
    __host__ __device__ Vec2f operator-(const float t);
    __host__ __device__ Vec2f operator*(const float scl);
    __host__ __device__ Vec2f operator/(const float scl);
};

struct Vec4f; // Forward declaration
struct Vec3f {
    float x, y, z;
    __host__ __device__ Vec3f();
    __host__ __device__ Vec3f(float x, float y, float z);
    __host__ __device__ Vec3f(float a);
    __host__ __device__ Vec4f toVec4f();

    // Basic operations
    __host__ __device__ Vec3f operator+(const Vec3f &v) const;
    __host__ __device__ Vec3f operator+(const float t) const;
    __host__ __device__ Vec3f operator-(const Vec3f &v) const;
    __host__ __device__ Vec3f operator-(const float t) const;
    __host__ __device__ Vec3f operator*(const float scl) const;
    __host__ __device__ Vec3f operator/(const float scl) const;
    __host__ __device__ void operator+=(const Vec3f &v);
    __host__ __device__ void operator-=(const Vec3f &v);
    __host__ __device__ void operator*=(const float scl);
    __host__ __device__ void operator/=(const float scl);
    // Advanced operations
    __host__ __device__ float operator*(const Vec3f &v) const; // Dot product
    __host__ __device__ Vec3f operator&(const Vec3f &v) const; // Cross product
    __host__ __device__ float mag(); // Magnitude
    __host__ __device__ void norm(); // Normalize
    // Special operations
    __host__ __device__ static Vec3f bary(Vec2f v, Vec2f v0, Vec2f v1, Vec2f v2); // Barycentric coordinates
    __host__ __device__ void limit(float min, float max); // Limit the vector

    // Transformations
    __host__ __device__ static Vec3f translate(Vec3f &v, const Vec3f &t);
    __host__ __device__ static Vec3f rotate(Vec3f &v, const Vec3f &o, const Vec3f &n, const float w);
    __host__ __device__ static Vec3f rotateX(Vec3f &v, const Vec3f &o, const float rx);
    __host__ __device__ static Vec3f rotateY(Vec3f &v, const Vec3f &o, const float ry);
    __host__ __device__ static Vec3f rotateZ(Vec3f &v, const Vec3f &o, const float rz);
    __host__ __device__ static Vec3f scale(Vec3f &v, const Vec3f &o, const Vec3f &scl);
    __host__ __device__ static Vec3f scale(Vec3f &v, const Vec3f &o, const float scl);
    // Transformations but on self
    __host__ __device__ void translate(const Vec3f &t);
    __host__ __device__ void rotateX(const Vec3f &o, const float rx);
    __host__ __device__ void rotateY(const Vec3f &o, const float ry);
    __host__ __device__ void rotateZ(const Vec3f &o, const float rz);
    __host__ __device__ void scale(const Vec3f &o, const Vec3f &scl);
    __host__ __device__ void scale(const Vec3f &o, const float scl);
};

struct Vec4f {
    float x, y, z, w;
    __host__ __device__ Vec4f();
    __host__ __device__ Vec4f(float x, float y, float z, float w);
    __host__ __device__ Vec3f toVec3f(bool norm=true); // From Homogeneous to Cartesian

    __host__ __device__ Vec4f operator+(const Vec4f &v);
    __host__ __device__ Vec4f operator+(const float t);
    __host__ __device__ Vec4f operator-(const Vec4f &v);
    __host__ __device__ Vec4f operator-(const float t);
    __host__ __device__ Vec4f operator*(const float scl);
    __host__ __device__ Vec4f operator/(const float scl);

    __host__ __device__ void limit(float min, float max); // Limit the vector
};

struct Vec2ulli {
    ULLInt x = 0, y = 0;
};

// SoA structure Vecs

struct Vec1f_ptr {
    float *x;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec1f_ptr &vec);
    void setAll(float val);
};
struct Vec2f_ptr {
    float *x, *y;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec2f_ptr &vec);
    void setAll(float val);
};
struct Vec3f_ptr {
    float *x, *y, *z;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec3f_ptr &vec);
    void setAll(float val);
};
struct Vec4f_ptr {
    float *x, *y, *z, *w;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec4f_ptr &vec);
    void setAll(float val);
};

// Specific purposes Vecs
struct Vec1lli_ptr {
    LLInt *x;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec1lli_ptr &vec);
};
struct Vec2i_ptr {
    int *x, *y;
    ULLInt size = 0;

    void malloc(ULLInt size);
    void free();
    void operator+=(Vec2i_ptr &vec);
};

// Atomic functions for float
__device__ bool atomicMinFloat(float* addr, float value);
__device__ bool atomicMinDouble(double* addr, double value);

// Helpful kernels
__global__ void setFloatAllKernel(float *arr, float val, ULLInt size);
__global__ void setLLIntAllKernel(LLInt *arr, LLInt val, ULLInt size);

#endif