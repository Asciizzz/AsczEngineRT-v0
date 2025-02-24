#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <Vector.cuh>

struct TxtrPtr {
    int w, h, off;
};

class AsczTxtr {
public:
    // Host memory
    std::vector<Flt4> h_flat;
    std::vector<TxtrPtr> h_ptr;

    int appendTexture(const char *path);

    // Device memory
    Flt4 *d_flat; int size = 0;
    TxtrPtr *d_ptr; int count = 0;

    void freeDevice();
    void toDevice();
};

#endif