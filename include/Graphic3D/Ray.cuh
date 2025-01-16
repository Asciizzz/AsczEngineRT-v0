#ifndef RAY_CUH
#define RAY_CUH

#include <Vector.cuh>

class Ray {
public:
    Vec3f origin;
    Vec3f direction;

    __host__ __device__ Ray();
    __host__ __device__ Ray(Vec3f direction); // Origin at (0, 0, 0)
    __host__ __device__ Ray(Vec3f origin, Vec3f direction);

    // AABB intersection
    __host__ __device__ bool intersectAABB(Vec3f &AABBmin, Vec3f &AABBmax);

    __host__ __device__ Vec3f reflect(Vec3f normal);
};

#endif