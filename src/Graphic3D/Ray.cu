#include <Ray.cuh>

// Constructors
Ray::Ray() : origin(Vec3f(0, 0, 0)), direction(Vec3f(0, 0, 0)) {}   
Ray::Ray(Vec3f direction) : origin(Vec3f(0, 0, 0)), direction(direction) {}
Ray::Ray(Vec3f origin, Vec3f direction) : origin(origin), direction(direction) {}

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

// Reflection
Vec3f Ray::reflect(Vec3f normal) {
    return direction - normal * (2.0f * (direction * normal));
}