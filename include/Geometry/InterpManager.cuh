#ifndef INTERPMANAGER_CUH
#define INTERPMANAGER_CUH

#include <HitManager.cuh>

struct RayHit {
    bool hit = false;
    Vec3f vrtx;
    Vec2f txtr; // Will be ignored for now
    Vec3f nrml;
    Vec3f colr;
    float t;
};

class InterpManager {
public:
    static RayHit interpolate(
        const Geom& geom, const GeomHit& geomHit, const Vec3f& rayOrg, const Vec3f& rayDir
    ) {

    };

    static RayHit interpolateTriangle(
        const Triangle& triangle, const TriangleHit& triangleHit, const Vec3f& rayOrg, const Vec3f& rayDir
    ) {
        RayHit rayHit;

        rayHit.hit = triangleHit.hit;
        if (!rayHit.hit) return rayHit;

        rayHit.t = triangleHit.t;
        rayHit.vrtx = rayOrg + rayDir * rayHit.t;
        rayHit.nrml = triangle.n;
        rayHit.colr = triangle.color;

        return rayHit;
    }

}