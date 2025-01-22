#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>
#include <Sphere.cuh>
#include <Plane.cuh>

struct Geom {
    enum GeomType { TRIANGLE, SPHERE, PLANE } type;
    union {
        Triangle tri;
        Sphere sph;
        Plane pln;
    };

    Geom() {}
    Geom(GeomType type) : type(type) {}

    // === Test material attributes ===

    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    int txtrIdx = -1;

    // Some special attributes
    bool isSky = false;
};

#endif