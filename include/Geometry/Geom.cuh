#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>

enum class GeomType {
    TRIANGLE,
    SPHERE
};

struct Geom {
    GeomType type;
    union {
        Triangle triangle;
        // Sphere sphere;
    };
};

#endif