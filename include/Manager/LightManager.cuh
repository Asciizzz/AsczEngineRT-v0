#ifndef LIGHTMANAGER_CUH
#define LIGHTMANAGER_CUH

#include <Vector.cuh>

struct LightSrc {
    Flt3 pos;
    float size;
    float intens = 1.0f;
    Flt3 colr = Flt3(1.0f);
};

class LightManager {
public:
    std::vector<LightSrc> h_lSrc;

    LightSrc *d_lSrc;
    int num;

    LightManager() {}

    void appendLight(LightSrc light) {
        h_lSrc.push_back(light);
    }

    void toDevice() {
        num = h_lSrc.size();
        cudaMalloc(&d_lSrc, num * sizeof(LightSrc));
        cudaMemcpy(d_lSrc, h_lSrc.data(), num * sizeof(LightSrc), cudaMemcpyHostToDevice);
    }
};

#endif