#ifndef GEOM_CUH
#define GEOM_CUH

#include <Vector.cuh>
#define VecGeom std::vector<AzGeom>

struct AzGeom {
    enum Type { TRIANGLE, SPHERE } type;

    union {
        struct {
            Int3 v; // Triangle vertices indices
            Int3 t; // Triangle tcoords indices
            Int3 n; // Triangle normals indices
        } tri;
        struct {
            int c; // Sphere center index
            float r; // Sphere radius
        } sph;
    };

    int m = 0; // Material index

    AzGeom(Int3 tv, Int3 tt, Int3 tn, int m) :
        type(TRIANGLE), tri({ tv, tt, tn }), m(m) {}

    AzGeom(int c, float r, int m) :
        type(SPHERE), sph({ c, r }), m(m) {}
};

#endif