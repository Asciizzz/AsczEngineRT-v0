#ifndef PLANE_CUH
#define PLANE_CUH

#include <Vector.cuh>

struct Plane {
    Vec3f n;
    float d;

    Vec3f color;

    Plane() {}
    Plane(Vec3f n, float d, Vec3f color) :
        n(n), d(d), color(color) {}
};

#endif