#ifndef GEOM_CUH
#define GEOM_CUH

#include <Vector.cuh>

struct AszcTriangle {
    Int3 fv;
    Int3 ft;
    Int3 fn;
};

struct AsczSphere {
    float c;
    float r;
};

struct AsczGeom {
    enum Type { TRIANGLE, SPHERE } type;

    union {
        AszcTriangle tri;
        AsczSphere sph;
    };

    int mtl = -1;
};

#endif