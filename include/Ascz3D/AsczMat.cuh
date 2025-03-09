#ifndef AsczMat_CUH
#define AsczMat_CUH

#include <vector>
#include <string>

struct AzMtl {
    float Alb_r = 0.0f;
    float Alb_g = 0.0f;
    float Alb_b = 0.0f;
    int AlbMap = -1;

    float Rough = 1.0f;
    float Metal = 0.0f;
    // 0 < metal < 1 is physically impossible but AsczEngine allow it
    // for "dusty metal" or painted metal surfaces

    float Tr = 0.0f;
    float Ior = 1.0f;

    // Emission color and intensity
    float Ems_r = 0.0f;
    float Ems_g = 0.0f;
    float Ems_b = 0.0f;
    float Ems_i = 0.0f;

    // Debug values
    bool NoShade = false;
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
    ~AsczMat();

    // Host memory
    std::vector<std::wstring> names = { L"Default" };
    std::vector<std::wstring> paths = { L"AsczEngine" };
    std::vector<AzMtl> h_mtls = { AzMtl() };

    // Device memory
    AzMtl *d_mtls = nullptr;
    int mtlsNum = 1;

    int append(AzMtl mtl, std::wstring name, std::wstring path);
    void toDevice();
};

#endif