#ifndef RAY_CUH
#define RAY_CUH

#include <Geom.cuh>

struct RayHit {
    bool hit = false;
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
    float w = 0;
};

struct Ray {
    Vec3f origin;
    Vec3f direction;
    float Ni = 1.0f; // Refractive index

    __host__ __device__ Ray();
    __host__ __device__ Ray(Vec3f direction, float Ni=1.0f); // Origin at (0, 0, 0)
    __host__ __device__ Ray(Vec3f origin, Vec3f direction, float Ni=1.0f);

    // AABB intersection
    __host__ __device__ bool intersectAABB(Vec3f &AABBmin, Vec3f &AABBmax);

    __host__ __device__ Vec3f reflect(Vec3f normal);
    __host__ __device__ Vec3f refract(Vec3f normal, float Ni2);

    __host__ __device__ RayHit hitTriangle(Triangle tri);
    __host__ __device__ RayHit hitGeom(Geom geom);
};

#endif