#ifndef ASCZMTL_CUH
#define ASCZMTL_CUH

#include <Vector.cuh>

struct Material {
    // True mtl attributes
    Flt3 Ka = Flt3(0.1f);
    Flt3 Kd = Flt3(0.8f);
    Flt3 Ks = Flt3(0.3f);
    int mKd = -1;

    float Ns = 10.0f;
    float Tr = 0.0f;

    // Not in a usual mtl file
    float reflect = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    bool noShade = false;
    bool noShadow = false;
};

class AsczMtl {
public:
    // Host memory
    std::vector<Material> h_mtls = { Material() };

    // Device memory
    Material *d_mtls = nullptr;
    int mtlsNum = 1;

    int appendMaterial(Material mtl);
    void freeDevice();
    void toDevice();
};

#endif