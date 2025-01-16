#ifndef TRIANGLE_CUH
#define TRIANGLE_CUH

#include <Vector.cuh>

struct TriangleHit {
    bool hit = false;
    Vec3f point;
    float u, v;
    float t;
};

struct Triangle {
    Vec3f v0, v1, v2;
    Vec3f n0, n1, n2;
    Vec2f t0, t1, t2;

    // Color, for the time being
    Vec3f c0, c1, c2;

    // For the time being material only contain texture map
    int mIdx = -1;
};

#endif