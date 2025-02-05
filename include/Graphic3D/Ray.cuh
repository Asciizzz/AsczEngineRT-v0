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
    Flt3 o;
    Flt3 d;
    Flt3 invd; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ni = 1.0f; // Refractive index

    _hst_dev_ Ray();
    _hst_dev_ Ray(Flt3 o, Flt3 d, float w=1.0f, float Ni=1.0f);

    _hst_dev_ Flt3 reflect(const Flt3 &n);
    _hst_dev_ Flt3 refract(const Flt3 &n, float Ni2);
};

#endif