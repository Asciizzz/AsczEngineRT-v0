#ifndef CAMERA_CUH
#define CAMERA_CUH

#include <string>
#include <Ray.cuh>

class Camera {
public:
    float fov = M_PI_2; // The most perfect fov imo

    Flt3 pos, rot;
    void restrictRot();

    Flt3 forward, right, up;
    void updateView();

    // Ray generation
    _hst_dev_
    Flt2 getScrnNDC(float x, float y, float width, float height);
    _hst_dev_
    Ray castRay(float x, float y, float width, float height);

    void update();

    // Some beta settings
    float mSens = 0.2f;
    float slowFactor = 0.2f;
    float fastFactor = 5.0f;
    float velSpec = 20;

    bool focus = true;

    // Debug
    std::string data();
};

#endif