#include <AsczCam.cuh>

#include <string>
#include <iostream>

// ================================ Ray ================================

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

// ================================ Camera ================================

void AsczCam::restrictRot() {
    if (rot.x <= -M_PI_2) rot.x = -M_PI_2 + 0.001;
    else if (rot.x >= M_PI_2) rot.x = M_PI_2 - 0.001;

    if (rot.y > M_PIx2) rot.y -= M_PIx2;
    else if (rot.y < 0) rot.y += M_PIx2;
}

void AsczCam::updateView() {
    frwd.x = sin(rot.y) * cos(rot.x);
    frwd.y = sin(rot.x);
    frwd.z = cos(rot.y) * cos(rot.x);
    frwd.norm();

    rght = Flt3(0, 1, 0) ^ frwd;
    rght.norm();

    up = frwd ^ rght;
    up.norm();
}

__device__
Ray AsczCam::castRay(float x, float y, float w, float h, float rnd1, float rnd2) const {
    float ndcX = (w - 2 * x) / w;
    float ndcY = (h - 2 * y) / h;

    float tanFov = tanf(fov / 2);

    Flt3 rD = frwd + rght * ndcX * tanFov * w / h + up * ndcY * tanFov;
    rD /= rD.x * rD.x + rD.y * rD.y + rD.z * rD.z;

    Flt3 focalPoint = pos + rD * focalDist;
    float r = aperture * sqrtf(rnd1);
    float theta = 2 * M_PI * rnd2;

    Flt3 apertureOffset = rght * r * cosf(theta) + up * r * sinf(theta);

    Flt3 rO = pos + apertureOffset;
    rD = (focalPoint - rO).norm();

    return Ray(rO, rD);
}


void AsczCam::update() {
    restrictRot();
    updateView();
}