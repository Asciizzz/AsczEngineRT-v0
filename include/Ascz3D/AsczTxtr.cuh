#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <vector>

class AsczTxtr {
public:
    // Host memory
    std::vector<float> h_tr = {0.0f};
    std::vector<float> h_tg = {0.0f};
    std::vector<float> h_tb = {0.0f};
    std::vector<float> h_ta = {0.0f};
    std::vector<int> h_tw = {0};
    std::vector<int> h_th = {0};
    std::vector<int> h_toff = {0};

    int appendTexture(const char *path);

    // Device memory
    float *d_tr = nullptr;
    float *d_tg = nullptr;
    float *d_tb = nullptr;
    float *d_ta = nullptr;
    int size = 1;

    int *d_tw = nullptr;
    int *d_th = nullptr;
    int *d_toff = nullptr;
    int count = 1;

    void freeDevice();
    void toDevice();
};

#endif