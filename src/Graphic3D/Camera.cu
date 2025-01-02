#include <Camera.cuh>

void Camera::restrictRot() {
    if (rot.x <= -M_PI_2) rot.x = -M_PI_2 + 0.001;
    else if (rot.x >= M_PI_2) rot.x = M_PI_2 - 0.001;

    if (rot.y > M_2_PI) rot.y -= M_2_PI;
    else if (rot.y < 0) rot.y += M_2_PI;
}

void Camera::updateView() {
    forward.x = sin(rot.y) * cos(rot.x);
    forward.y = sin(rot.x);
    forward.z = cos(rot.y) * cos(rot.x);
    forward.norm();

    right = Vec3f(0, 1, 0) & forward;
    right.norm();

    up = forward & right;
    up.norm();
}


Vec2f Camera::getScrnNDC(float x, float y, float width, float height) {
    // Note: w/2 and h/2 are used to center the screen space coordinates
    return Vec2f((2 * x - width) / width, (height - 2 * y) / height);
}

Ray Camera::castRay(float x, float y, float width, float height) {
    // Step 1: Convert screen space coordinates to NDC
    Vec2f ndc = getScrnNDC(x, y, width, height);

    // Step 2: Calculate the direction vector for the ray
    // tan(fov / 2) scales the direction based on the field of view.
    // Aspect ratio adjusts the direction for non-square screens (i.e., when width != height).
    float tanFov = tan(fov / 2);
    Vec3f rayDir = forward * tanFov + right * ndc.x * tanFov * width / height + up * ndc.y * tanFov;

    // Step 3: Normalize the direction vector
    rayDir.norm();

    // Step 4: Create and return the ray from the camera's position and the calculated direction
    return Ray(pos, rayDir);
}


void Camera::update() {
    restrictRot();
    updateView();
}

// Debug
std::string Camera::data() {
    std::string str = "Camera\n";
    str += "| Pos: " + std::to_string(pos.x) + ", " + std::to_string(pos.y) + ", " + std::to_string(pos.z) + "\n";
    str += "| Rot: " + std::to_string(rot.x) + ", " + std::to_string(rot.y) + ", " + std::to_string(rot.z) + "\n";
    str += "| Fd: " + std::to_string(forward.x) + ", " + std::to_string(forward.y) + ", " + std::to_string(forward.z) + "\n";
    str += "| Rg: " + std::to_string(right.x) + ", " + std::to_string(right.y) + ", " + std::to_string(right.z) + "\n";
    str += "| Up: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) + "\n";
    str += "| Fov: " + std::to_string(fov * 180 / M_PI) + "\n";
    return str;
}