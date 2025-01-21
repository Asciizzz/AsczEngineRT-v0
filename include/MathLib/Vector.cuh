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

#define _hst_dev_ __host__ __device__

struct Vec2f {
    float x, y;
    _hst_dev_ Vec2f();
    _hst_dev_ Vec2f(float x, float y);
    
    // Bracket operator
    _hst_dev_ float& operator[](int i);

    // Basic operations
    _hst_dev_ Vec2f operator+(const Vec2f &vec);
    _hst_dev_ Vec2f operator+(const float t);
    _hst_dev_ Vec2f operator-(const Vec2f &vec);
    _hst_dev_ Vec2f operator-(const float t);
    _hst_dev_ Vec2f operator*(const float scl);
    _hst_dev_ Vec2f operator/(const float scl);
};

struct Vec4f; // Forward declaration
struct Vec3f {
    float x, y, z;
    _hst_dev_ Vec3f();
    _hst_dev_ Vec3f(float x, float y, float z);
    _hst_dev_ Vec3f(float a);
    _hst_dev_ Vec4f toVec4f();

    // Bracket operator
    _hst_dev_ float& operator[](int i);

    // Basic operations
    _hst_dev_ Vec3f operator-() const;
    _hst_dev_ Vec3f operator+(const Vec3f &v) const;
    _hst_dev_ Vec3f operator+(const float t) const;
    _hst_dev_ Vec3f operator-(const Vec3f &v) const;
    _hst_dev_ Vec3f operator-(const float t) const;
    _hst_dev_ Vec3f operator*(const float scl) const;
    _hst_dev_ Vec3f operator/(const float scl) const;
    _hst_dev_ void operator+=(const Vec3f &v);
    _hst_dev_ void operator-=(const Vec3f &v);
    _hst_dev_ void operator*=(const float scl);
    _hst_dev_ void operator/=(const float scl);
    // Dot product
    _hst_dev_ float operator*(const Vec3f &v) const;
    // Cross product 
    _hst_dev_ Vec3f operator&(const Vec3f &v) const;
    // Magnitude
    _hst_dev_ float mag();
    // Normalize
    _hst_dev_ void norm();
    // Barycentric coordinates
    _hst_dev_ static Vec3f bary(Vec2f v, Vec2f v0, Vec2f v1, Vec2f v2);
    _hst_dev_ static Vec3f bary(Vec3f v, Vec3f v0, Vec3f v1, Vec3f v2);
    // Limit the vector
    _hst_dev_ void limit(float min, float max);

    // Transformations
    _hst_dev_ static Vec3f translate(Vec3f &v, const Vec3f &t);
    _hst_dev_ static Vec3f rotate(Vec3f &v, const Vec3f &o, const Vec3f &n, const float w); // RIght-hand rule
    _hst_dev_ static Vec3f rotateX(Vec3f &v, const Vec3f &o, const float rx);
    _hst_dev_ static Vec3f rotateY(Vec3f &v, const Vec3f &o, const float ry);
    _hst_dev_ static Vec3f rotateZ(Vec3f &v, const Vec3f &o, const float rz);
    _hst_dev_ static Vec3f scale(Vec3f &v, const Vec3f &o, const Vec3f &scl);
    _hst_dev_ static Vec3f scale(Vec3f &v, const Vec3f &o, const float scl);
    // Transformations but on self
    _hst_dev_ void translate(const Vec3f &t);
    _hst_dev_ void rotateX(const Vec3f &o, const float rx);
    _hst_dev_ void rotateY(const Vec3f &o, const float ry);
    _hst_dev_ void rotateZ(const Vec3f &o, const float rz);
    _hst_dev_ void scale(const Vec3f &o, const Vec3f &scl);
    _hst_dev_ void scale(const Vec3f &o, const float scl);
};

struct Vec4f {
    float x, y, z, w;
    _hst_dev_ Vec4f();
    _hst_dev_ Vec4f(float x, float y, float z, float w);
    _hst_dev_ Vec3f toVec3f(bool norm=true);
    
    // Bracket operator
    _hst_dev_ float& operator[](int i);

    // Basic operations
    _hst_dev_ Vec4f operator+(const Vec4f &v);
    _hst_dev_ Vec4f operator+(const float t);
    _hst_dev_ Vec4f operator-(const Vec4f &v);
    _hst_dev_ Vec4f operator-(const float t);
    _hst_dev_ Vec4f operator*(const float scl);
    _hst_dev_ Vec4f operator/(const float scl);

    // Limit the vector
    _hst_dev_ void limit(float min, float max);
};

#endif