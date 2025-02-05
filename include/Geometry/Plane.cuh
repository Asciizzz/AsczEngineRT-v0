#ifndef PLANE_CUH
#define PLANE_CUH

#include <Vector.cuh>

struct Plane {
    Flt3 n;
    float d;

    Flt3 color;

    Plane() {}
    Plane(Flt3 n, float d, Flt3 color) :
        n(n), d(d), color(color) {}
};

#endif