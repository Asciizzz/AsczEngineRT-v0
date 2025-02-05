#ifndef RAY_CUH
#define RAY_CUH

#include <Vector.cuh>
#include <curand_kernel.h>

struct RayHit {
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

struct Ray {
    Flt3 o;
    Flt3 d;
    Flt3 invd; // Inverse direction for AABB intersection since division is a b*tch
    float w = 1.0f; // Weight
    float Ni = 1.0f; // Refractive index

    _hst_dev_ Ray();
    _hst_dev_ Ray(Flt3 o, Flt3 d, float w=1.0f, float Ni=1.0f);

    _hst_dev_ Flt3 reflect(const Flt3 &n);
    _hst_dev_ Flt3 refract(const Flt3 &n, float Ni2);

    // Generate a random point on a unit disk using cosine-weighted sampling
    static __device__ Flt3 randomInUnitDisk(curandState *randState) {
        float theta = curand_uniform(randState) * 2.0f * M_PI;
        float r = sqrt(curand_uniform(randState)); // Ensures uniform sampling
        return Flt3(r * cosf(theta), r * sinf(theta), 0.0f); 
    }

    // Generate a random ray slightly angled from the original ray
    static __device__ Ray generateJitteredRay(const Ray &ray, float jitter, curandState *randState) {
        // Create an orthonormal basis around the ray direction
        Flt3 up = fabs(ray.d.z) < 0.99f ? Flt3(0, 0, 1) : Flt3(1, 0, 0);
        Flt3 right = ray.d & up;  // Cross product to get tangent vector
        right.norm();
        Flt3 upVec = right & ray.d; // Get second perpendicular vector

        // Sample random offset in disk
        Flt3 offset = randomInUnitDisk(randState);
        Flt3 jitteredDir = ray.d + right * offset.x * jitter + upVec * offset.y * jitter;
        jitteredDir.norm();

        return Ray(ray.o, jitteredDir, ray.w, ray.Ni);
    }
};

#endif