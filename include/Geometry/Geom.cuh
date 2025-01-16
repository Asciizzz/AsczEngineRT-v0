#ifndef GEOM_CUH
#define GEOM_CUH

#include <Triangle.cuh>
#include <Sphere.cuh>

enum GeomType {
    TRIANGLE,
    SPHERE
};

struct GeomHit {
    GeomType type;
    union {
        TriangleHit triangle;
        SphereHit sphere;
    };

    GeomHit() { type = TRIANGLE; }

    static bool checkHit(const GeomHit &hit) {
        switch (hit.type) {
            case TRIANGLE: return hit.triangle.hit;
            case SPHERE:   return hit.sphere.hit;
        }
    }

    static bool checkHitCloser(const GeomHit &hit, const GeomHit &other) {
        if (!checkHit(hit)) return false;
        if (!checkHit(other)) return true;

        switch (hit.type) {
            case TRIANGLE: return hit.triangle.t < other.triangle.t;
            case SPHERE:   return hit.sphere.t < other.sphere.t;
        }
    }
};

struct Geom {
    GeomType type;
    union {
        Triangle triangle;
        Sphere sphere;
    };
};

#endif