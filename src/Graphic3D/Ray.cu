#include <Ray.cuh>

// Constructors
Ray::Ray() {};
Ray::Ray(Flt3 o, Flt3 d, float w, float Ni) :
    o(o), d(d), invd(1.0f / d), w(w), Ni(Ni) {};

// Reflection + refraction
Flt3 Ray::reflect(const Flt3 &n) {
    return d - n * (2.0f * (d * n));
}

Flt3 Ray::refract(const Flt3 &n, float Ni2) {
    float Ni1 = Ni;
    float cosI = -n * d;
    float cosT2 = 1.0f - Ni1 * Ni1 * (1.0f - cosI * cosI) / (Ni2 * Ni2);

    if (cosT2 < 0.0f) return Flt3(); // Total internal reflection

    return d * Ni1 / Ni2 + n * (Ni1 * cosI / Ni2 - sqrt(cosT2));
}

Flt3 Ray::reflect(const Flt3 &d, const Flt3 &n) {
    return d - n * (2.0f * (d * n));
}