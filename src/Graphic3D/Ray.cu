#include <Ray.cuh>

// Constructors
Ray::Ray() : origin(Vec3f()), direction(Vec3f()) {}   
Ray::Ray(Vec3f direction, float Ni) : direction(direction), Ni(Ni) {}
Ray::Ray(Vec3f origin, Vec3f direction, float Ni) :
    origin(origin), direction(direction), Ni(Ni) {}

// Bounding volume intersections

bool Ray::intersectAABB(Vec3f &AABBmin, Vec3f &AABBmax) {
    float tmin = (AABBmin.x - origin.x) / direction.x;
    float tmax = (AABBmax.x - origin.x) / direction.x;

    if (tmin > tmax) {
        float temp = tmin;
        tmin = tmax;
        tmax = temp;
    }

    float tymin = (AABBmin.y - origin.y) / direction.y;
    float tymax = (AABBmax.y - origin.y) / direction.y;

    if (tymin > tymax) {
        float temp = tymin;
        tymin = tymax;
        tymax = temp;
    }
    
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    float tzmin = (AABBmin.z - origin.z) / direction.z;
    float tzmax = (AABBmax.z - origin.z) / direction.z;

    if (tzmin > tzmax) {
        float temp = tzmin;
        tzmin = tzmax;
        tzmax = temp;
    }

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    return tmax >= tmin && tmax >= 0.0f;
}

// Reflection + refraction
Vec3f Ray::reflect(Vec3f normal) {
    return direction - normal * (2.0f * (direction * normal));
}

Vec3f Ray::refract(Vec3f normal, float Ni2) {
    float Ni1 = Ni;
    float cosI = -normal * direction;
    float cosT2 = 1.0f - Ni1 * Ni1 * (1.0f - cosI * cosI) / (Ni2 * Ni2);

    if (cosT2 < 0.0f) return Vec3f(); // Total internal reflection

    return direction * Ni1 / Ni2 + normal * (Ni1 * cosI / Ni2 - sqrt(cosT2));
}


// Intersection

RayHit Ray::hitTriangle(Triangle tri) {
    RayHit hit;

    Vec3f e1 = tri.v1 - tri.v0;
    Vec3f e2 = tri.v2 - tri.v0;
    Vec3f h = direction & e2;
    float a = e1 * h;

    if (a > -0.00001 && a < 0.00001) return hit;

    float f = 1.0f / a;
    Vec3f s = origin - tri.v0;
    float u = f * (s * h);

    if (u < 0.0f || u > 1.0f) return hit;

    Vec3f q = s & e1;
    float v = f * (direction * q);

    if (v < 0.0f || u + v > 1.0f) return hit;

    float t = f * (e2 * q);

    if (t > 0.00001) {
        hit.hit = true;
        hit.t = t;
        hit.u = u;
        hit.v = v;
        hit.w = 1 - u - v;
    }

    return hit;
}

RayHit Ray::hitGeom(Geom geom) {
    RayHit hit;

    switch (geom.type) {
        case Geom::TRIANGLE: hit = hitTriangle(geom.triangle); break;
        default: break;
    }

    return hit;
}