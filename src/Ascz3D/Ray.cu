#include <Ray.cuh>

// Constructors
Ray::Ray() {};
Ray::Ray(Flt3 o, Flt3 d, float w, float Ior, int ignore) :
    o(o), d(d), invd(1.0f / d), w(w), Ior(Ior), ignore(ignore) {}

// Reflection + refraction
Flt3 Ray::reflect(const Flt3 &n) {
    return d - n * (2.0f * (d * n));
}

Flt3 Ray::refract(const Flt3 &n, float Ior2) {
    float Ior1 = Ior;
    float cosI = -n * d;
    float cosT2 = 1.0f - Ior1 * Ior1 * (1.0f - cosI * cosI) / (Ior2 * Ior2);

    if (cosT2 < 0.0f) return Flt3(); // Total internal reflection

    return d * Ior1 / Ior2 + n * (Ior1 * cosI / Ior2 - sqrt(cosT2));
}

Flt3 Ray::reflect(const Flt3 &d, const Flt3 &n) {
    return d - n * (2.0f * (d * n));
}