#ifndef VECTOR_CUH
#define VECTOR_CUH

#include <iostream>
#include <vector>
#include <cstdio>
#include <cmath>

/*
Take a screenshot of these abomination below
to instantly kill a programmer on the spot
*/

#define VecI std::vector<int>
#define VecF std::vector<float>

#define VecUI32t std::vector<uint32_t>

#define Vec2f std::vector<Flt2>
#define Vec3f std::vector<Flt3>
#define Vec4f std::vector<Flt4>

#define Vec3i std::vector<Int3>

#define _glb_ __global__
#define _dev_ __device__
#define _hst_ __host__
#define _hst_dev_ __host__ __device__

constexpr float M_PI = 3.14159265358979323846; // 180 degrees
constexpr float M_PI_2 = 1.57079632679489661923; // 90 degrees
constexpr float M_2_PI = 6.28318530717958647692; // 360 degrees
constexpr float M_E = 2.71828182845904523536;

constexpr float EPSILON_1 = 1e-3;
constexpr float EPSILON_2 = 1e-6;
constexpr int THREADS_PER_BLOCK = 256;

struct Flt2 {
    float x, y;
    _hst_dev_ Flt2();
    _hst_dev_ Flt2(float x, float y);
    
    // Bracket operator
    _hst_dev_ float& operator[](int i);
    _hst_dev_ float operator[](int i) const;

    // Basic operations
    _hst_dev_ Flt2 operator+(const Flt2 &vec) const;
    _hst_dev_ Flt2 operator+(const float t) const;
    _hst_dev_ Flt2 operator-(const Flt2 &vec) const;
    _hst_dev_ Flt2 operator-(const float t) const;
    _hst_dev_ Flt2 operator*(const float scl) const;
    _hst_dev_ Flt2 operator/(const float scl) const;
};

struct Flt3 {
    float x, y, z;
    _hst_dev_ Flt3();
    _hst_dev_ Flt3(float x, float y, float z);
    _hst_dev_ Flt3(float a);

    // Bracket operator
    _hst_dev_ float& operator[](int i);
    _hst_dev_ float operator[](int i) const;

    // Basic operations
    _hst_dev_ Flt3 operator-() const;
    _hst_dev_ Flt3 operator+(const Flt3 &v) const;
    _hst_dev_ Flt3 operator+(const float t) const;
    _hst_dev_ Flt3 operator-(const Flt3 &v) const;
    _hst_dev_ Flt3 operator-(const float t) const;
    _hst_dev_ Flt3 operator*(const float scl) const;
    _hst_dev_ Flt3 operator/(const float scl) const;

    _hst_dev_ Flt3 operator+=(const Flt3 &v);
    _hst_dev_ Flt3 operator+=(const float t);
    _hst_dev_ Flt3 operator-=(const Flt3 &v);
    _hst_dev_ Flt3 operator-=(const float t);
    _hst_dev_ Flt3 operator*=(const float scl);
    _hst_dev_ Flt3 operator/=(const float scl);

    _hst_dev_ friend Flt3 operator+(const float t, const Flt3 &v);
    _hst_dev_ friend Flt3 operator-(const float t, const Flt3 &v);
    _hst_dev_ friend Flt3 operator*(const float scl, const Flt3 &v);
    _hst_dev_ friend Flt3 operator/(const float scl, const Flt3 &v);

    // Comparison
    _hst_dev_ bool operator==(const Flt3 &v) const;
    _hst_dev_ bool operator!=(const Flt3 &v) const;

    // Element-wise multiplication
    _hst_dev_ Flt3 operator&(const Flt3 &v) const;
    _hst_dev_ Flt3 operator&=(const Flt3 &v);
    // Dot product
    _hst_dev_ float operator*(const Flt3 &v) const;
    // Cross product 
    _hst_dev_ Flt3 operator^(const Flt3 &v) const;

    // Magnitude
    _hst_dev_ float mag();
    // Normalize
    _hst_dev_ Flt3 norm();
    // Absolute value
    _hst_dev_ void abs();

    // Transformations
    _hst_dev_ static Flt3 translate(Flt3 &v, const Flt3 &t);
    _hst_dev_ static Flt3 rotate(Flt3 &v, const Flt3 &o, const Flt3 &n, const float w); // RIght-hand rule
    _hst_dev_ static Flt3 rotateX(Flt3 &v, const Flt3 &o, const float rx);
    _hst_dev_ static Flt3 rotateY(Flt3 &v, const Flt3 &o, const float ry);
    _hst_dev_ static Flt3 rotateZ(Flt3 &v, const Flt3 &o, const float rz);
    _hst_dev_ static Flt3 scale(Flt3 &v, const Flt3 &o, const Flt3 &scl);
    _hst_dev_ static Flt3 scale(Flt3 &v, const Flt3 &o, const float scl);

    // Transformations but on self
    _hst_dev_ void translate(const Flt3 &t);
    _hst_dev_ void rotateX(const Flt3 &o, const float rx);
    _hst_dev_ void rotateY(const Flt3 &o, const float ry);
    _hst_dev_ void rotateZ(const Flt3 &o, const float rz);
    _hst_dev_ void scale(const Flt3 &o, const Flt3 &scl);
    _hst_dev_ void scale(const Flt3 &o, const float scl);
};

struct Flt4 {
    float x, y, z, w;
    _hst_dev_ Flt4();
    _hst_dev_ Flt4(float x, float y, float z, float w);
    _hst_dev_ Flt3 f3(bool norm = false);
    
    // Bracket operator
    _hst_dev_ float& operator[](int i);
    _hst_dev_ float operator[](int i) const;

    // Basic operations
    _hst_dev_ Flt4 operator+(const Flt4 &v) const;
    _hst_dev_ Flt4 operator+(const float t) const;
    _hst_dev_ Flt4 operator-(const Flt4 &v) const;
    _hst_dev_ Flt4 operator-(const float t) const;
    _hst_dev_ Flt4 operator*(const float scl) const;
    _hst_dev_ Flt4 operator/(const float scl) const;
};

struct Int3 {
    int x, y, z;
    _hst_dev_ Int3();
    _hst_dev_ Int3(int x, int y, int z);
    _hst_dev_ Int3(int a);

    // Bracket operator
    _hst_dev_ int& operator[](int i);

    _hst_dev_ Int3 operator+(const Int3 &v) const;
    _hst_dev_ Int3 operator+(const int t) const;
    _hst_dev_ void operator+=(const Int3 &v);
    _hst_dev_ void operator+=(const int t);

    _hst_dev_ Int3 operator-(const Int3 &v) const;
    _hst_dev_ Int3 operator-(const int t) const;
    _hst_dev_ void operator-=(const Int3 &v);
    _hst_dev_ void operator-=(const int t);
};

#endif