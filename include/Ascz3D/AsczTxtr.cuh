#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <Vector.cuh>

class AsczTxtr {
public:
    // Host memory
    VecF h_tr = {0.0f};
    VecF h_tg = {0.0f};
    VecF h_tb = {0.0f};
    VecF h_ta = {0.0f};
    VecI h_tw = {0};
    VecI h_th = {0};
    VecI h_toff = {0};

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