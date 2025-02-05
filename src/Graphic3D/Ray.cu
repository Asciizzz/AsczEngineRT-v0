#include <Ray.cuh>

// Constructors
Ray::Ray() {};
Ray::Ray(Vec3f o, Vec3f d, float w, float Ni) :
    o(o), d(d), invd(1.0f / d), w(w), Ni(Ni) {};

// Reflection + refraction
Vec3f Ray::reflect(const Vec3f &n) {
    return d - n * (2.0f * (d * n));
}

Vec3f Ray::refract(const Vec3f &n, float Ni2) {
    float Ni1 = Ni;
    float cosI = -n * d;
    float cosT2 = 1.0f - Ni1 * Ni1 * (1.0f - cosI * cosI) / (Ni2 * Ni2);

    if (cosT2 < 0.0f) return Vec3f(); // Total internal reflection

    return d * Ni1 / Ni2 + n * (Ni1 * cosI / Ni2 - sqrt(cosT2));
}