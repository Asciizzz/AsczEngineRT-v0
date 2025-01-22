#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <Vector.cuh>

struct Triangle {
    Vec3f v0, v1, v2;
    Vec2f t0, t1, t2;
    Vec3f n0, n1, n2;
    Vec3f c0, c1, c2; // Color

    Triangle() {}
    Triangle(Vec3f v1, Vec3f v2, Vec3f v3) :
        v0(v1), v1(v2), v2(v3) {}

    // Some helper functions
    void uniformColor(Vec3f color) {
        c0 = color;
        c1 = color;
        c2 = color;
    }
    void uniformNormal(Vec3f normal) {
        n0 = normal;
        n1 = normal;
        n2 = normal;
    }
    void normAll() {
        n0.norm();
        n1.norm();
        n2.norm();
    }

    void scale(Vec3f o, float s) {
        v0.scale(o, s);
        v1.scale(o, s);
        v2.scale(o, s);
    }

    void rotate(Vec3f o, Vec3f n, float w) {
        v0 = Vec3f::rotate(v0, o, n, w);
        v1 = Vec3f::rotate(v1, o, n, w);
        v2 = Vec3f::rotate(v2, o, n, w);

        n0 = Vec3f::rotate(n0, Vec3f(), n, w);
        n1 = Vec3f::rotate(n1, Vec3f(), n, w);
        n2 = Vec3f::rotate(n2, Vec3f(), n, w);
        normAll();
    }

    void translate(Vec3f t) {
        v0 += t;
        v1 += t;
        v2 += t;
    }
};

#endif