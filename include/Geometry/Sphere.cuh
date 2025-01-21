#ifndef SPHERE_CUH
#define SPHERE_CUH

#include <Vector.cuh>

struct Sphere {
    Vec3f o;
    float r;

    bool invert = false; // Invert the normal

    Vec3f color;
};

#endif