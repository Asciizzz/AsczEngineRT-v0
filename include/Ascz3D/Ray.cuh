#ifndef RAY_CUH
#define RAY_CUH

#include <Vector.cuh>

struct Ray {
    Flt3 o;
    Flt3 d;
    Flt3 invd; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ni = 1.0f; // Refractive index

    __host__ __device__ Ray();
    __host__ __device__ Ray(Flt3 o, Flt3 d, float w=1.0f, float Ni=1.0f);

    __host__ __device__ Flt3 reflect(const Flt3 &n);
    __host__ __device__ Flt3 refract(const Flt3 &n, float Ni2);

    static __host__ __device__ Flt3 reflect(const Flt3 &d, const Flt3 &n);
};

#endif