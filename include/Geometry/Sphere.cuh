#ifndef SPHERE_CUH
#define SPHERE_CUH

#include <Vector.cuh>

struct Sphere {
    Flt3 o;
    float r;

    Flt3 color;
    bool invert = false; // Invert the normal

    Sphere() {}
    Sphere(Flt3 o, float r, Flt3 color=Flt3()) :
        o(o), r(r), color(color) {}
};

#endif