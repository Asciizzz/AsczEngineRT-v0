#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <Vector.cuh>

struct Triangle {
    Flt3 v0, v1, v2;
    Flt2 t0, t1, t2;
    Flt3 n0, n1, n2;
    Flt3 c0, c1, c2; // Color

    Triangle() {}
    Triangle(Flt3 v0, Flt3 v1, Flt3 v2) :
        v0(v0), v1(v1), v2(v2) {}

    // Some helper functions
    void uniformColor(Flt3 color) {
        c0 = color;
        c1 = color;
        c2 = color;
    }
    void uniformNormal(Flt3 normal) {
        n0 = normal;
        n1 = normal;
        n2 = normal;
    }
    void normAll() {
        n0.norm();
        n1.norm();
        n2.norm();
    }

    void scale(Flt3 o, float s) {
        v0.scale(o, s);
        v1.scale(o, s);
        v2.scale(o, s);
    }

    void rotate(Flt3 o, Flt3 n, float w) {
        v0 = Flt3::rotate(v0, o, n, w);
        v1 = Flt3::rotate(v1, o, n, w);
        v2 = Flt3::rotate(v2, o, n, w);

        n0 = Flt3::rotate(n0, Flt3(), n, w);
        n1 = Flt3::rotate(n1, Flt3(), n, w);
        n2 = Flt3::rotate(n2, Flt3(), n, w);
        normAll();
    }

    void translate(Flt3 t) {
        v0 += t;
        v1 += t;
        v2 += t;
    }
};

#endif