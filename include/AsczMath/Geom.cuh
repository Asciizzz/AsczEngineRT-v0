#ifndef GEOM_CUH
#define GEOM_CUH

#include <Vector.cuh>
#define VecGeom std::vector<AzGeom>

struct AzGeom {
    enum Type { TRIANGLE, SPHERE } type;

    union {
        struct {
            Int3 v; // Triangle vertices
            Int3 t; // Triangle texture coordinates
            Int3 n; // Triangle normals
        } tri;
        struct {
            Flt3 c; // Sphere center
            float r; // Sphere radius
        } sph;
    };

    int m = 0; // Material index

    AzGeom(Int3 tv, Int3 tt, Int3 tn, int m) :
        type(TRIANGLE), tri({ tv, tt, tn }), m(m) {}

    AzGeom(Flt3 sc, float sr, int m) :
        type(SPHERE), sph({ sc, sr }), m(m) {}
};

#endif