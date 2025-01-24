#ifndef MATMANAGER_CUH
#define MATMANAGER_CUH

#include <Material.cuh>
#include <Vector.cuh>

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