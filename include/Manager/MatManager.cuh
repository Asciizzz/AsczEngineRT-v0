#ifndef MATMANAGER_CUH
#define MATMANAGER_CUH

#include <Vector.cuh>

struct Material {
    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    // Some special attributes
    bool isSky = false;

    // Some actually real attributes
    Vec3f Kd = Vec3f(0.5f);
    int mapKd = -1;
};

class MatManager {
public:
    // Host memory
    std::vector<Material> h_mats;
    // Append material and return the index
    int appendMaterial(Material mat);
    // Get the last mesh
    Material &getLastMaterial() { return h_mats.back(); }

    // Device memory
    Material *d_mats = nullptr;
    int matsNum = 0;

    void freeDevice();
    void hostToDevice();
};

#endif