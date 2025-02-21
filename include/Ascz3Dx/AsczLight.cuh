#ifndef ASCZLIGHT_CUH
#define ASCZLIGHT_CUH

#include <Vector.cuh>

struct LightSrc {
    Flt3 pos;
    Flt3 colr = Flt3(1.0f);
    float intens = 1.0f;

    // Light falloff
    bool falloff = true; // Use falloff or not
    float bias = 1.0f; // The start of the falloff
    float exp = 2.0f; // Control how intensity falls off with distance
    float falloffDist = 100.0f; // Distance for exponential falloff
};

class AsczLight {
public:
    std::vector<LightSrc> h_lSrc;

    LightSrc *d_lSrc;
    int num;

    AsczLight() {}

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