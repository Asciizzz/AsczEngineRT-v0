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

    if (rot.y > M_2_PI) rot.y -= M_2_PI;
    else if (rot.y < 0) rot.y += M_2_PI;
}

void AsczCam::updateView() {
    forward.x = sin(rot.y) * cos(rot.x);
    forward.y = sin(rot.x);
    forward.z = cos(rot.y) * cos(rot.x);
    forward.norm();

    right = Flt3(0, 1, 0) ^ forward;
    right.norm();

    up = forward ^ right;
    up.norm();
}


Flt2 AsczCam::getScrnNDC(float x, float y, float width, float height) const {
    // Note: w/2 and h/2 are used to center the screen space coordinates
    return Flt2((2 * x - width) / width, (height - 2 * y) / height);
}

Ray AsczCam::castRay(float x, float y, float width, float height, float dx, float dy) const {
    // Step 1: Convert screen space coordinates to NDC
    Flt2 ndc = getScrnNDC(x, y, width, height);

    // Step 2: Calculate the direction vector for the ray
    // tan(fov / 2) scales the direction based on the field of view.
    // Aspect ratio adjusts the direction for non-square screens (i.e., when width != height).
    float tanFov = tan(fov / 2);
    Flt3 rayDir = forward + right * ndc.x * tanFov * width / height + up * ndc.y * tanFov;
    rayDir += right * dx + up * dy;

    // Step 3: Normalize the direction vector
    rayDir.norm();

    // Step 4: Create and return the ray from the camera's position and the calculated direction
    return Ray(pos, rayDir);
}


void AsczCam::update() {
    restrictRot();
    updateView();
}