#include <AsczCam.cuh>

#include <string>
#include <iostream>

// ================================ Ray ================================

__device__ Ray::Ray() {};
__device__ Ray::Ray(Flt3 o, Flt3 d, float w, float Ior, int ignore) :
    o(o), d(d), invd(1.0f / d), w(w), Ior(Ior), ignore(ignore) {}

// ================================ Camera ================================

void AsczCam::restrictRot() {
    if (rpit <= -M_PI_2) rpit = -M_PI_2 + 0.001;
    else if (rpit >= M_PI_2) rpit = M_PI_2 - 0.001;

    if (ryaw > M_PIx2) ryaw -= M_PIx2;
    else if (ryaw < 0) ryaw += M_PIx2;
}

void AsczCam::updateView() {
    fw_x = sin(ryaw) * cos(rpit);
    fw_y = sin(rpit);
    fw_z = cos(ryaw) * cos(rpit);

    float fw_mag = sqrt(fw_x * fw_x + fw_y * fw_y + fw_z * fw_z);
    fw_x /= fw_mag, fw_y /= fw_mag, fw_z /= fw_mag;

    // right = (0, 1, 0) cross forward;
    rg_x = -fw_y;
    rg_y =  fw_x;
    rg_z = -fw_x;

    float rg_mag = sqrt(rg_x * rg_x + rg_y * rg_y + rg_z * rg_z);
    rg_x /= rg_mag, rg_y /= rg_mag, rg_z /= rg_mag;

    // up = frwd ^ rght;
    // up.norm();

    up_x = fw_y * rg_z - fw_z * rg_y;
    up_y = fw_z * rg_x - fw_x * rg_z;
    up_z = fw_x * rg_y - fw_y * rg_x;

    float up_mag = sqrt(up_x * up_x + up_y * up_y + up_z * up_z);
    up_x /= up_mag, up_y /= up_mag, up_z /= up_mag;
}

__device__
Ray AsczCam::castRay(float x, float y, float w, float h, float r1, float r2) const {
    float ndcX = (w - 2 * x) / w;
    float ndcY = (h - 2 * y) / h;

    float tanFov = tanf(fov / 2);
    float w_h = w / h;

    // Flt3 rD = frwd + (rght*ndcX*w_h + up*ndcY) * tanFov;

    float rD_x = fw_x + (rg_x * ndcX * w_h + up_x * ndcY) * tanFov;
    float rD_y = fw_y + (rg_y * ndcX * w_h + up_y * ndcY) * tanFov;
    float rD_z = fw_z + (rg_z * ndcX * w_h + up_z * ndcY) * tanFov;
    float rD_mag = sqrt(rD_x * rD_x + rD_y * rD_y + rD_z * rD_z);
    rD_x /= rD_mag, rD_y /= rD_mag, rD_z /= rD_mag;

    // r1, r2 for random aperture
    float r = aperture * sqrtf(r1);
    float theta = M_PIx2 * r2;

    // Flt3 rO = pos + (rght*cosf(theta) + up*sinf(theta)) * r;

    float sinTheta = sinf(theta);
    float cosTheta = cosf(theta);

    float rO_x = px + (rg_x * cosTheta + up_x * sinTheta) * r;
    float rO_y = py + (rg_y * cosTheta + up_y * sinTheta) * r;
    float rO_z = pz + (rg_z * cosTheta + up_z * sinTheta) * r;


    // To make sure the focal point is exactly on the focal plane
    // And not on the radius of a sphere
    float RdotF = rD_x * fw_x + rD_y * fw_y + rD_z * fw_z;

    // Flt3 focalPoint = pos + rD * focalDist / RdotF;

    float fD_RdotF = focalDist / RdotF;
    float focal_x = px + rD_x * fD_RdotF;
    float focal_y = py + rD_y * fD_RdotF;
    float focal_z = pz + rD_z * fD_RdotF;

    // Anti-aliasing
    // focalPoint += (rght * (r2 - 0.5f) + up * (r1 - 0.5f)) * 0.004f;

    float r3 = (r2 - 0.5f) * 0.004f;
    float r4 = (r1 - 0.5f) * 0.004f;
    focal_x += rg_x * r3 + up_x * r4;
    focal_y += rg_y * r3 + up_y * r4;
    focal_z += rg_z * r3 + up_z * r4;




    // rD = (focalPoint - rO).norm();

    rD_x = focal_x - rO_x;
    rD_y = focal_y - rO_y;
    rD_z = focal_z - rO_z;
    rD_mag = sqrt(rD_x * rD_x + rD_y * rD_y + rD_z * rD_z);
    rD_x /= rD_mag, rD_y /= rD_mag, rD_z /= rD_mag;

    // r3, r4 for antialiasing

    return Ray(Flt3(rO_x, rO_y, rO_z), Flt3(rD_x, rD_y, rD_z));
}


void AsczCam::update() {
    restrictRot();
    updateView();
}