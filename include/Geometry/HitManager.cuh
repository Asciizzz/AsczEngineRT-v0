#ifndef HITMANAGER_CUH
#define HITMANAGER_CUH

#include <Geom.cuh>

class HitManager {
public:
    static GeomHit intersect(Geom geom, Vec3f origin, Vec3f direction) {
        GeomHit geomHit;
        geomHit.type = geom.type;

        switch (geom.type) {
            case TRIANGLE:
                geomHit.triangle = intersectTriangle(geom.triangle, origin, direction);
                break;
            case SPHERE:
                geomHit.sphere = intersectSphere(geom.sphere, origin, direction);
                break;
        }

        return geomHit;
    }

    static TriangleHit intersectTriangle(Triangle triangle, Vec3f origin, Vec3f direction) {
        TriangleHit newHit; // Sound like a new boy band

        Vec3f e1 = triangle.v1 - triangle.v0;
        Vec3f e2 = triangle.v2 - triangle.v0;
        Vec3f h = direction & e2;
        float a = e1 * h;

        if (a > -0.00001 && a < 0.00001) return newHit;

        float f = 1.0f / a;
        Vec3f s = origin - triangle.v0;
        float u = f * (s * h);

        if (u < 0.0f || u > 1.0f) return newHit;

        Vec3f q = s & e1;
        float v = f * (direction * q);

        if (v < 0.0f || u + v > 1.0f) return newHit;

        float t = f * (e2 * q); 

        if (t > 0.00001) {
            newHit.hit = true;
            newHit.t = t;
            newHit.u = u;
            newHit.v = v;
        }

        return newHit;
    }

    static SphereHit intersectSphere(Sphere sphere, Vec3f origin, Vec3f direction) {
        SphereHit newHit;

        Vec3f oc = origin - sphere.c;
        float a = direction * direction;
        float b = 2.0f * (oc * direction);
        float c = oc * oc - sphere.r * sphere.r;
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) return newHit;

        float t = (-b - sqrt(discriminant)) / (2.0f * a);

        if (t < 0.00001) return newHit;

        newHit.hit = true;
        newHit.t = t;
        newHit.point = origin + direction * t;

        return newHit;
    }
};

#endif