#ifndef ASCZCAM_CUH
#define ASCZCAM_CUH

#include <Vector.cuh>

struct Ray {
    Flt3 o;
    Flt3 d;
    Flt3 invd; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ior = 1.0f; // Refractive index

    int ignore = -1; // Index of the object to ignore

    __host__ __device__ Ray();
    __host__ __device__ Ray(Flt3 o, Flt3 d, float w=1.0f, float Ior=1.0f, int ignore=-1);

    __host__ __device__ Flt3 reflect(const Flt3 &n);
    __host__ __device__ Flt3 refract(const Flt3 &n, float Ior2);

    static __host__ __device__ Flt3 reflect(const Flt3 &d, const Flt3 &n);
};



class AsczCam {
public:
    float fov = M_PI_2; // The most perfect fov imo

    Flt3 pos, rot;
    void restrictRot();

    Flt3 forward, right, up;
    void updateView();

    // Ray generation
    __host__ __device__
    Flt2 getScrnNDC(float x, float y, float width, float height) const;
    __host__ __device__
    Ray castRay(float x, float y, float width, float height) const;

    void update();

    // Some beta settings
    float mSens = 0.2f;
    float slowFactor = 0.2f;
    float fastFactor = 5.0f;
    float velSpec = 20;

    bool focus = true;
};

#endif