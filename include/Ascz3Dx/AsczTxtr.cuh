#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <Vector.cuh>

struct TxPtr {
    int w, h, off;
};

class AsczTxtr {
public:
    // Host memory
    std::vector<Flt4> h_flat;
    std::vector<TxPtr> h_ptr;

    int appendTexture(const char *path);

    // Device memory
    Flt4 *d_flat; int size = 0;
    TxPtr *d_ptr; int count = 0;

    void freeDevice();
    void toDevice();
};

#endif