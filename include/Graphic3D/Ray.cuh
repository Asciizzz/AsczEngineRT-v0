#ifndef RAY_CUH
#define RAY_CUH

#include <Geom.cuh>

struct RayHit {
    bool hit = false;
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

struct Ray {
    Vec3f origin;
    Vec3f direction;
    float Ni = 1.0f; // Refractive index

    __host__ __device__ Ray();
    __host__ __device__ Ray(Vec3f direction, float Ni=1.0f); // Origin at (0, 0, 0)
    __host__ __device__ Ray(Vec3f origin, Vec3f direction, float Ni=1.0f);

    // AABB intersection
    __device__ bool intersectAABB(const Vec3f &AABBmin, const Vec3f &AABBmax);

    __device__ Vec3f reflect(const Vec3f &normal);
    __device__ Vec3f refract(const Vec3f &normal, float Ni2);

    // Special geometry intersection
    __device__ Vec3f triParamTUV(const Triangle &tri);
    __device__ float sphParamT(const Sphere &sph);
    __device__ float plnParamT(const Plane &pln);
};

#endif