#ifndef MATMANAGER_CUH
#define MATMANAGER_CUH

#include <Vector.cuh>

struct Material {
    // True mtl attributes
    Vec3f Kd = Vec3f(0.5f);
    int mapKd = -1;

    // Not in a usual mtl file
    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    bool Phong = false;
};

class MatManager {
public:
    // Host memory
    std::vector<Material> h_mats = { Material() };
    // Append material and return the index
    int appendMaterial(Material mat);
    // Get the last mesh
    Material &getLastMaterial() { return h_mats.back(); }

    // Device memory
    Material *d_mats = nullptr;
    int matsNum = 0;

    void freeDevice();
    void toDevice();
};

#endif