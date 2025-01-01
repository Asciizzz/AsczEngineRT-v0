#include <Ray.cuh>

// Constructors
Ray::Ray() : origin(Vec3f(0, 0, 0)), direction(Vec3f(0, 0, 0)) {}   
Ray::Ray(Vec3f direction) : origin(Vec3f(0, 0, 0)), direction(direction) {}
Ray::Ray(Vec3f origin, Vec3f direction) : origin(origin), direction(direction) {}

// Bounding volume intersections

bool Ray::intersectAABB(Vec3f &AABBmin, Vec3f &AABBmax) {
    // So you can throw it into a for loop
    float o[3] = {origin.x, origin.y, origin.z};    
    float d[3] = {direction.x, direction.y, direction.z};
    float min[3] = {AABBmin.x, AABBmin.y, AABBmin.z};
    float max[3] = {AABBmax.x, AABBmax.y, AABBmax.z};   

    // Initializing tmin and tmax arrays to store the intersection t-values for each axis
    float tmin[3] = {
        (min[0] - o[0]) / d[0],
        (min[1] - o[1]) / d[1],
        (min[2] - o[2]) / d[2]
    };
    
    float tmax[3] = {
        (max[0] - o[0]) / d[0],
        (max[1] - o[1]) / d[1],
        (max[2] - o[2]) / d[2]
    };

    // Handling the case where direction is zero (parallel rays)
    for (int i = 0; i < 3; i++) {
        if (d[i] == 0.0f) {
            // If the ray is parallel to the AABB along this axis, we skip this axis
            if (o[i] < min[i] || o[i] > max[i]) {
                return false;  // No intersection, since the ray is parallel and outside the box
            }
            // Otherwise, set tmin and tmax to extreme values for this axis
            tmin[i] = -std::numeric_limits<float>::infinity();
            tmax[i] = std::numeric_limits<float>::infinity();
        }
    }

    // Swapping tmin and tmax if tmin > tmax
    for (int i = 0; i < 3; i++) {
        if (tmin[i] > tmax[i]) {
            float temp = tmin[i];
            tmin[i] = tmax[i];
            tmax[i] = temp;
        }
    }

    // Finding the overall entry and exit t-values
    float tEntry = fmaxf(fmaxf(tmin[0], tmin[1]), tmin[2]);
    float tExit = fminf(fminf(tmax[0], tmax[1]), tmax[2]);

    // The ray intersects if tEntry < tExit and the ray enters in front of the AABB (tExit > 0)
    return tEntry < tExit && tExit > 0;
}

// Reflection
Vec3f Ray::reflect(Vec3f normal) {
    return direction - normal * 2 * (direction * normal);
}

// Refraction
Vec3f Ray::refract(Vec3f normal, float eta) {
    float cosI = -normal * direction;
    float sinT2 = eta * eta * (1.0f - cosI * cosI);
    if (sinT2 > 1.0f) return Vec3f(0, 0, 0);  // Total internal reflection
    return direction * eta + normal * (eta * cosI - sqrtf(1.0f - sinT2));
}