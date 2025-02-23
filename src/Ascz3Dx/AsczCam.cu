#include <AsczCam.cuh>

#include <string>
#include <iostream>

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


Flt2 AsczCam::getScrnNDC(float x, float y, float width, float height) {
    // Note: w/2 and h/2 are used to center the screen space coordinates
    return Flt2((2 * x - width) / width, (height - 2 * y) / height);
}

Ray AsczCam::castRay(float x, float y, float width, float height) {
    // Step 1: Convert screen space coordinates to NDC
    Flt2 ndc = getScrnNDC(x, y, width, height);

    // Step 2: Calculate the direction vector for the ray
    // tan(fov / 2) scales the direction based on the field of view.
    // Aspect ratio adjusts the direction for non-square screens (i.e., when width != height).
    float tanFov = tan(fov / 2);
    Flt3 rayDir = forward + right * ndc.x * tanFov * width / height + up * ndc.y * tanFov;

    // Step 3: Normalize the direction vector
    rayDir.norm();

    // Step 4: Create and return the ray from the camera's position and the calculated direction
    return Ray(pos, rayDir);
}


void AsczCam::update() {
    restrictRot();
    updateView();
}