#include <AsczCam.cuh>

#include <string>
#include <iostream>

// ================================ Ray ================================

__device__ Ray::Ray() {};
__device__ Ray::Ray(float ox, float oy, float oz, float dx, float dy, float dz, float w, float Ior, int ignore) :
    ox(ox), oy(oy), oz(oz), dx(dx), dy(dy), dz(dz), w(w), Ior(Ior), ignore(ignore) {
    rdx = 1.0f / dx;
    rdy = 1.0f / dy;
    rdz = 1.0f / dz;
}

// ================================ Camera ================================

void AsczCam::restrictRot() {
    rpit = fminf(fmaxf(rpit, EPSILON_2-M_PI_2), M_PI_2-EPSILON_2);
    ryaw = fmodf(ryaw, M_PIx2);
}

void AsczCam::updateView() {
    // Forward vector
    fw_x = sin(ryaw) * cos(rpit);
    fw_y = sin(rpit);
    fw_z = cos(ryaw) * cos(rpit);

    float fw_mag = sqrt(fw_x * fw_x + fw_y * fw_y + fw_z * fw_z);
    fw_x /= fw_mag, fw_y /= fw_mag, fw_z /= fw_mag;

    // Right vector (<0, 1, 0> cross forward)
    rg_x = fw_z;  // rx = 1 * fw_z - 0 * fw_y;
    rg_y = 0;     // ry = 0 * fw_x - 0 * fw_z;
    rg_z = -fw_x; // rz = 0 * fw_y - 1 * fw_x;

    float rg_mag = sqrt(rg_x * rg_x + rg_y * rg_y + rg_z * rg_z);
    rg_x /= rg_mag, rg_y /= rg_mag, rg_z /= rg_mag;

    // Up vector (forward cross right)
    up_x = fw_y * rg_z - fw_z * rg_y;
    up_y = fw_z * rg_x - fw_x * rg_z;
    up_z = fw_x * rg_y - fw_y * rg_x;

    float up_mag = sqrt(up_x * up_x + up_y * up_y + up_z * up_z);
    up_x /= up_mag, up_y /= up_mag, up_z /= up_mag;
}

__device__
Ray AsczCam::castRay(float x, float y, float w, float h, float r1, float r2) const {
    float ndcX = (w - 2 * x) / h;
    float ndcY = (h - 2 * y) / h;

    float tanFov = tanf(fov / 2);

    // Find perfect direction
    float rD_x = fw_x + (rg_x * ndcX + up_x * ndcY) * tanFov;
    float rD_y = fw_y + (rg_y * ndcX + up_y * ndcY) * tanFov;
    float rD_z = fw_z + (rg_z * ndcX + up_z * ndcY) * tanFov;

    float rD_mag = sqrt(rD_x * rD_x + rD_y * rD_y + rD_z * rD_z);
    rD_x /= rD_mag, rD_y /= rD_mag, rD_z /= rD_mag;

    // r1, r2 for random aperture
    float r = aperture * sqrtf(r1);
    float theta = M_PIx2 * r2;

    // Offset origin for the aperture
    float sinTheta = sinf(theta);
    float cosTheta = cosf(theta);

    float rO_x = px + (rg_x * cosTheta + up_x * sinTheta) * r;
    float rO_y = py + (rg_y * cosTheta + up_y * sinTheta) * r;
    float rO_z = pz + (rg_z * cosTheta + up_z * sinTheta) * r;


    // Make sure the focal point is exactly on the focal plane
    float RdotF = rD_x * fw_x + rD_y * fw_y + rD_z * fw_z;
    float fD_RdotF = focalDist / RdotF;

    float focal_x = px + rD_x * fD_RdotF;
    float focal_y = py + rD_y * fD_RdotF;
    float focal_z = pz + rD_z * fD_RdotF;

    // Anti-aliasing
    float r3 = (r2 - 0.5f) * 0.004f;
    float r4 = (r1 - 0.5f) * 0.004f;

    focal_x += rg_x * r3 + up_x * r4;
    focal_y += rg_y * r3 + up_y * r4;
    focal_z += rg_z * r3 + up_z * r4;

    // The new direction
    rD_x = focal_x - rO_x;
    rD_y = focal_y - rO_y;
    rD_z = focal_z - rO_z;

    rD_mag = sqrt(rD_x * rD_x + rD_y * rD_y + rD_z * rD_z);
    rD_x /= rD_mag, rD_y /= rD_mag, rD_z /= rD_mag;

    return Ray(rO_x, rO_y, rO_z, rD_x, rD_y, rD_z);
}


void AsczCam::update() {
    restrictRot();
    updateView();
}