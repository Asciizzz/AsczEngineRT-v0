#include <AsczCam.cuh>

#include <string>
#include <iostream>

// ================================ Ray ================================

__device__ Ray::Ray() {};
__device__ Ray::Ray(Flt3 o, Flt3 d, float w, float Ior, int ignore) :
    o(o), d(d), invd(1.0f / d), w(w), Ior(Ior), ignore(ignore) {}

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
    float w_h = w / h;

    Flt3 rD = frwd + (rght*ndcX*w_h + up*ndcY) * tanFov;
    rD /= rD.x * rD.x + rD.y * rD.y + rD.z * rD.z;

    float r = aperture * sqrtf(rnd1);
    float theta = 2 * M_PI * rnd2;

    Flt3 rO = pos + (rght*cosf(theta) + up*sinf(theta)) * r;

    Flt3 focalPoint = pos + rD * focalDist;
    rD = (focalPoint - rO).norm();

    return Ray(rO, rD);
}


void AsczCam::update() {
    restrictRot();
    updateView();
}