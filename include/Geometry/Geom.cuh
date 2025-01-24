#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>
#include <Sphere.cuh>
#include <Plane.cuh>

#include <Material.cuh>

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

    int mat = 0; // Material index
};

#endif