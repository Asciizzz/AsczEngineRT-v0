#ifndef ASCZCAM_CUH
#define ASCZCAM_CUH

#include <Ray.cuh>

class AsczCam {
public:
    float fov = M_PI_2; // The most perfect fov imo

    Flt3 pos, rot;
    void restrictRot();

    Flt3 forward, right, up;
    void updateView();

    // Ray generation
    __host__ __device__
    Flt2 getScrnNDC(float x, float y, float width, float height);
    __host__ __device__
    Ray castRay(float x, float y, float width, float height);

    void update();

    // Some beta settings
    float mSens = 0.2f;
    float slowFactor = 0.2f;
    float fastFactor = 5.0f;
    float velSpec = 20;

    bool focus = true;

    // Debug
    void debug();
};

#endif