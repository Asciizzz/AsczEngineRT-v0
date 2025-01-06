#ifndef RAYTRACING_CUH
#define RAYTRACING_CUH

#include <Ray.cuh>
#include <MatRtxFlags.cuh>

class RayTracing {
public:
    static Ray computeReflectionRay(Vec3f &vrtx, Vec3f &normal, Vec3f &dir);
};

#endif