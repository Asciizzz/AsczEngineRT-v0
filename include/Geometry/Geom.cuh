#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>
#include <Sphere.cuh>
#include <Plane.cuh>

struct Geom {
    enum GeomType { TRIANGLE, SPHERE, PLANE } type;
    union {
        Triangle triangle;
        Sphere sphere;
        Plane plane;
    };

    // === Test material attributes ===

    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    int txtrIdx = -1;

    // Some special attributes
    bool isSky = false;

    Geom() {}
};

#endif