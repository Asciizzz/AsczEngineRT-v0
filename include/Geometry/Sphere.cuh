#ifndef SPHERE_CUH
#define SPHERE_CUH

#include <Vector.cuh>

struct Sphere {
    Vec3f o;
    float r;

    Vec3f color;
    bool invert = false; // Invert the normal

    Sphere() {}
    Sphere(Vec3f o, float r, Vec3f color=Vec3f()) :
        o(o), r(r), color(color) {}
};

#endif