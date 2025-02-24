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

constexpr float M_PI = 3.14159265358979323846; // 180 degrees
constexpr float M_PI_2 = 1.57079632679489661923; // 90 degrees
constexpr float M_2_PI = 6.28318530717958647692; // 360 degrees
constexpr float M_E = 2.71828182845904523536;

constexpr float EPSILON_1 = 1e-3;
constexpr float EPSILON_2 = 1e-6;
constexpr int THREADS_PER_BLOCK = 256;

struct Flt2 {
    float x, y;
    __host__ __device__ Flt2();
    __host__ __device__ Flt2(float x, float y);
    
    // Bracket operator
    __host__ __device__ float& operator[](int i);
    __host__ __device__ float operator[](int i) const;

    // Basic operations
    __host__ __device__ Flt2 operator+(const Flt2 &vec) const;
    __host__ __device__ Flt2 operator+(const float t) const;
    __host__ __device__ Flt2 operator-(const Flt2 &vec) const;
    __host__ __device__ Flt2 operator-(const float t) const;
    __host__ __device__ Flt2 operator*(const float scl) const;
    __host__ __device__ Flt2 operator/(const float scl) const;
};

struct alignas(16) Flt3 {
    float x, y, z, pad;
    __host__ __device__ Flt3();
    __host__ __device__ Flt3(float x, float y, float z);
    __host__ __device__ Flt3(float a);

    // Bracket operator
    __host__ __device__ float& operator[](int i);
    __host__ __device__ float operator[](int i) const;

    // Basic operations
    __host__ __device__ Flt3 operator-() const;
    __host__ __device__ Flt3 operator+(const Flt3 &v) const;
    __host__ __device__ Flt3 operator+(const float t) const;
    __host__ __device__ Flt3 operator-(const Flt3 &v) const;
    __host__ __device__ Flt3 operator-(const float t) const;
    __host__ __device__ Flt3 operator*(const float scl) const;
    __host__ __device__ Flt3 operator/(const float scl) const;

    __host__ __device__ Flt3 operator+=(const Flt3 &v);
    __host__ __device__ Flt3 operator+=(const float t);
    __host__ __device__ Flt3 operator-=(const Flt3 &v);
    __host__ __device__ Flt3 operator-=(const float t);
    __host__ __device__ Flt3 operator*=(const float scl);
    __host__ __device__ Flt3 operator/=(const float scl);

    __host__ __device__ friend Flt3 operator+(const float t, const Flt3 &v);
    __host__ __device__ friend Flt3 operator-(const float t, const Flt3 &v);
    __host__ __device__ friend Flt3 operator*(const float scl, const Flt3 &v);
    __host__ __device__ friend Flt3 operator/(const float scl, const Flt3 &v);

    // Comparison
    __host__ __device__ bool operator==(const Flt3 &v) const;
    __host__ __device__ bool operator!=(const Flt3 &v) const;

    // Element-wise multiplication
    __host__ __device__ Flt3 operator&(const Flt3 &v) const;
    __host__ __device__ Flt3 operator&=(const Flt3 &v);
    // Dot product
    __host__ __device__ float operator*(const Flt3 &v) const;
    // Cross product 
    __host__ __device__ Flt3 operator^(const Flt3 &v) const;

    // Magnitude
    __host__ __device__ float mag();
    // Normalize
    __host__ __device__ Flt3 norm();
    // Absolute value
    __host__ __device__ void abs();

    // Clamping
    __host__ __device__ Flt3 clamp(const Flt3 &min, const Flt3 &max);
    __host__ __device__ Flt3 clamp(float min, float max);

    // Power
    __host__ __device__ Flt3 pow(float p);

    // Other
    __host__ __device__ bool isZero() const;

    // Transformations
    __host__ __device__ static Flt3 translate(Flt3 &v, const Flt3 &t);
    __host__ __device__ static Flt3 rotate(Flt3 &v, const Flt3 &o, const Flt3 &n, const float w); // RIght-hand rule
    __host__ __device__ static Flt3 rotateX(Flt3 &v, const Flt3 &o, const float rx);
    __host__ __device__ static Flt3 rotateY(Flt3 &v, const Flt3 &o, const float ry);
    __host__ __device__ static Flt3 rotateZ(Flt3 &v, const Flt3 &o, const float rz);
    __host__ __device__ static Flt3 scale(Flt3 &v, const Flt3 &o, const Flt3 &scl);
    __host__ __device__ static Flt3 scale(Flt3 &v, const Flt3 &o, const float scl);

    // Transformations but on self
    __host__ __device__ void translate(const Flt3 &t);
    __host__ __device__ void rotateX(const Flt3 &o, const float rx);
    __host__ __device__ void rotateY(const Flt3 &o, const float ry);
    __host__ __device__ void rotateZ(const Flt3 &o, const float rz);
    __host__ __device__ void scale(const Flt3 &o, const Flt3 &scl);
    __host__ __device__ void scale(const Flt3 &o, const float scl);
};

struct Flt4 {
    float x, y, z, w;
    __host__ __device__ Flt4();
    __host__ __device__ Flt4(float x, float y, float z, float w);
    __host__ __device__ Flt3 f3(bool norm = false);
    
    // Bracket operator
    __host__ __device__ float& operator[](int i);
    __host__ __device__ float operator[](int i) const;

    // Basic operations
    __host__ __device__ Flt4 operator+(const Flt4 &v) const;
    __host__ __device__ Flt4 operator+(const float t) const;
    __host__ __device__ Flt4 operator-(const Flt4 &v) const;
    __host__ __device__ Flt4 operator-(const float t) const;
    __host__ __device__ Flt4 operator*(const float scl) const;
    __host__ __device__ Flt4 operator/(const float scl) const;
};

struct Int3 {
    int x, y, z;
    __host__ __device__ Int3();
    __host__ __device__ Int3(int x, int y, int z);
    __host__ __device__ Int3(int a);

    // Bracket operator
    __host__ __device__ int& operator[](int i);

    __host__ __device__ Int3 operator+(const Int3 &v) const;
    __host__ __device__ Int3 operator+(const int t) const;
    __host__ __device__ void operator+=(const Int3 &v);
    __host__ __device__ void operator+=(const int t);

    __host__ __device__ Int3 operator-(const Int3 &v) const;
    __host__ __device__ Int3 operator-(const int t) const;
    __host__ __device__ void operator-=(const Int3 &v);
    __host__ __device__ void operator-=(const int t);
};

#endif