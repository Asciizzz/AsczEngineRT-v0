#ifndef SPHERE_CUH
#define SPHERE_CUH

#include <Vector.cuh>

struct SphereHit {
    bool hit = false;
    Vec3f point;
    float t;
};

struct Sphere {
    Vec3f c;
    float r;

    Vec3f color;

    // For the time being material only contain texture map
    int mIdx = -1;
};

#endif