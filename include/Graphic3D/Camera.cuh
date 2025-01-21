#ifndef CAMERA_CUH
#define CAMERA_CUH

#include <string>
#include <Ray.cuh>

class Camera {
public:
    float fov = M_PI_2;

    Vec3f pos, rot;
    void restrictRot();

    Vec3f forward, right, up;
    void updateView();

    // Ray generation
    _hst_dev_
    Vec2f getScrnNDC(float x, float y, float width, float height);
    _hst_dev_
    Ray castRay(float x, float y, float width, float height);

    void update();

    // BETA: movement
    Vec3f vel;

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