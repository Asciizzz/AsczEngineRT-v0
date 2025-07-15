#ifndef ASCZCAM_CUH
#define ASCZCAM_CUH

#include <AzDevMath.cuh>

struct Ray {
    float ox, oy, oz; // Origin
    float dx, dy, dz; // Direction
    float rdx, rdy, rdz; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ior = 1.0f; // Refractive index
    int ignore = -1; // Index of the object to ignore

    __device__ Ray();
    __host__ __device__ Ray(
        float ox, float oy, float oz,
        float dx, float dy, float dz,
        float w=1.0f, float Ior=1.0f, int ignore=-1
    );
};



class AsczCam {
public:
    float fov = M_PI_2; // The most perfect fov imo

    float px = 0.0f;
    float py = 0.0f;
    float pz = 0.0f;
    float rpit = 0.0f; // To look up and down
    float ryaw = 0.0f; // To look around

    void restrictRot();

    float fw_x = 0.0f, fw_y = 0.0f, fw_z = 0.0f;
    float rg_x = 0.0f, rg_y = 0.0f, rg_z = 0.0f;
    float up_x = 0.0f, up_y = 0.0f, up_z = 0.0f;

    void updateView();

    float aperture = 0.0f;
    float focalDist = 1.0f;

    // Ray generation
    __device__ Ray castRay(float x, float y, float w, float h, float r1=0.0f, float r2=0.0f) const;

    void update();

    // Some beta settings
    float mSens = 0.2f;
    float slowFactor = 0.2f;
    float fastFactor = 5.0f;
    float velSpec = 20;

    bool focus = true;
};

#endif