#ifndef VECTOR_CUH
#define VECTOR_CUH

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
    __host__ __device__ Vec3f operator-() const;
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
    // Dot product
    __host__ __device__ float operator*(const Vec3f &v) const;
    // Cross product 
    __host__ __device__ Vec3f operator&(const Vec3f &v) const;
    // Magnitude
    __host__ __device__ float mag();
    // Normalize
    __host__ __device__ void norm();
    // Barycentric coordinates
    __host__ __device__ static Vec3f bary(Vec2f v, Vec2f v0, Vec2f v1, Vec2f v2);
    __host__ __device__ static Vec3f bary(Vec3f v, Vec3f v0, Vec3f v1, Vec3f v2);
    // Limit the vector
    __host__ __device__ void limit(float min, float max);

    // Transformations
    __host__ __device__ static Vec3f translate(Vec3f &v, const Vec3f &t);
    __host__ __device__ static Vec3f rotate(Vec3f &v, const Vec3f &o, const Vec3f &n, const float w); // RIght-hand rule
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
    __host__ __device__ Vec3f toVec3f(bool norm=true);

    // Basic operations
    __host__ __device__ Vec4f operator+(const Vec4f &v);
    __host__ __device__ Vec4f operator+(const float t);
    __host__ __device__ Vec4f operator-(const Vec4f &v);
    __host__ __device__ Vec4f operator-(const float t);
    __host__ __device__ Vec4f operator*(const float scl);
    __host__ __device__ Vec4f operator/(const float scl);

    // Limit the vector
    __host__ __device__ void limit(float min, float max);
};

#endif