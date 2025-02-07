#ifndef GEOM_CUH
#define GEOM_CUH

#include <Vector.cuh>

struct AzTriangle {
    Int3 fv;
    Int3 ft;
    Int3 fn;
};

struct AzSphere {
    Flt3 c;
    float r;
};

struct AzGeom {
    enum Type { TRIANGLE, SPHERE } type = TRIANGLE;

    union {
        AzTriangle tri;
        AzSphere sph;
    };

    int m = -1; // Material index
};

#endif