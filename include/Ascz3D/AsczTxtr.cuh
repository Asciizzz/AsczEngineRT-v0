#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <Vector.cuh>

class AsczTxtr {
public:
    // Host memory
    VecF h_tr;
    VecF h_tg;
    VecF h_tb;
    VecF h_ta;
    VecI h_tw;
    VecI h_th;
    VecI h_toff;

    int appendTexture(const char *path);

    // Device memory
    float *d_tr = nullptr;
    float *d_tg = nullptr;
    float *d_tb = nullptr;
    float *d_ta = nullptr;
    int size = 0;

    int *d_tw = nullptr;
    int *d_th = nullptr;
    int *d_toff = nullptr;
    int count = 0;

    void freeDevice();
    void toDevice();
};

#endif