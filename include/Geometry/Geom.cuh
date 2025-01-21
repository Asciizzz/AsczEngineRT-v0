#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>
#include <Sphere.cuh>

struct Geom {
    enum GeomType { TRIANGLE, SPHERE } type;
    union {
        Triangle triangle;
        Sphere sphere;
    };

    // Test material attributes
    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    Geom() {}
};

#endif