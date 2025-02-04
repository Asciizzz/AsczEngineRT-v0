#ifndef RAY_CUH
#define RAY_CUH

#include <Vector.cuh>

struct RayHit {
    bool hit = false;
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

struct Ray {
    Vec3f o;
    Vec3f d;
    Vec3f invd; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ni = 1.0f; // Refractive index

    _hst_dev_ Ray();
    _hst_dev_ Ray(Vec3f d, float Ni=1.0f); // Origin at (0, 0, 0)
    _hst_dev_ Ray(Vec3f o, Vec3f d, float Ni=1.0f);

    _hst_dev_ Vec3f reflect(const Vec3f &n);
    _hst_dev_ Vec3f refract(const Vec3f &n, float Ni2);
};

#endif