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

    _hst_dev_ Ray();
    _hst_dev_ Ray(Vec3f direction, float Ni=1.0f); // Origin at (0, 0, 0)
    _hst_dev_ Ray(Vec3f origin, Vec3f direction, float Ni=1.0f);

    // AABB intersection
    _hst_dev_ bool intersectAABB(const Vec3f &AABBmin, const Vec3f &AABBmax);

    _hst_dev_ Vec3f reflect(const Vec3f &normal);
    _hst_dev_ Vec3f refract(const Vec3f &normal, float Ni2);
};

#endif