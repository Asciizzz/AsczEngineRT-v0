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

    __device__ Ray();
    __device__ Ray(Flt3 o, Flt3 d, float w=1.0f, float Ior=1.0f, int ignore=-1);
};



class AsczCam {
public:
    float fov = M_PI_2; // The most perfect fov imo

    Flt3 pos, rot;
    void restrictRot();

    Flt3 frwd, rght, up;
    void updateView();

    float aperture = 0.0f;
    float focalDist = 1.0f;

    // Ray generation
    __device__ Ray castRay(float x, float y, float w, float h, float rnd1=0.0f, float rnd2=0.0f) const;

    void update();

    // Some beta settings
    float mSens = 0.2f;
    float slowFactor = 0.2f;
    float fastFactor = 5.0f;
    float velSpec = 20;

    bool focus = true;
};

#endif