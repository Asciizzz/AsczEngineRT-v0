#ifndef AsczMat_CUH
#define AsczMat_CUH

#include <AzStruct.cuh>

/* Ior lookup table
Air: 1.000
Water: 1.333
Oil: 1.47   
Glass: 1.52
Diamond: 2.42
*/

#include <string>

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