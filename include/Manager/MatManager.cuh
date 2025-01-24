#ifndef MATMANAGER_CUH
#define MATMANAGER_CUH

#include <Vector.cuh>

struct Material {
    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    int txtrIdx = -1;

    // Some special attributes
    bool isSky = false;
};

class MatManager {
public:
    // Host memory
    std::vector<Material> h_mats;
    int appendMaterial(Material mat);

    // Device memory
    Material *d_mats = nullptr;
    int matsNum = 0;

    void freeDevice();
    void hostToDevice();
};

#endif