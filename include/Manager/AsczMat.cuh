#ifndef AsczMat_CUH
#define AsczMat_CUH

#include <Vector.cuh>

struct Material {
    // True mtl attributes
    Flt3 Ka = Flt3(0.1f);
    Flt3 Kd = Flt3(0.8f);
    Flt3 Ks = Flt3(0.5f);
    int mKd = -1;

    float Ns = 10.0f;
    float Tr = 0.0f;

    // Not in a usual mtl file
    float reflect = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;
};

class AsczMat {
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