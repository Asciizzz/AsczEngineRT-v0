#ifndef AsczMat_CUH
#define AsczMat_CUH

#include <Vector.cuh>

struct Material {
    Flt3 Alb = 0.8f;
    int AlbMap = -1;

    float Rough = 0.5f;
    float Metal = 0.0f;
    Flt3 Ems = 0.0f;

    // Transmission
    float Tr = 0.0f;
    float Ior = 1.0f;
};

/* Ior lookup table
Air: 1.000
Water: 1.333
Oil: 1.47   
Glass: 1.52
Diamond: 2.42
*/

class AsczMat {
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