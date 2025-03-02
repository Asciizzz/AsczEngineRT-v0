#ifndef AsczMat_CUH
#define AsczMat_CUH

#include <Vector.cuh>

struct AzMtl {
    Flt3 Alb = 0.8f;
    int AlbMap = -1;

    float Rough = 0.5f;
    float Metal = 0.0f;

    float Tr = 0.0f;
    float Ior = 1.0f;

    Flt3 Ems = 0.0f;
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
    std::vector<AzMtl> h_mtls = { AzMtl() };

    // Device memory
    AzMtl *d_mtls = nullptr;
    int mtlsNum = 1;

    int appendMaterial(AzMtl mtl);
    void freeDevice();
    void toDevice();
};

#endif