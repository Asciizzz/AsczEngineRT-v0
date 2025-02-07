#ifndef ASCZTXTR_CUH
#define ASCZTXTR_CUH

#include <Vector.cuh>

struct TxtrPtr {
    int w, h, off;
};

class AsczTxtr {
public:
    // Host memory
    std::vector<Flt4> h_txtrFlat;
    std::vector<TxtrPtr> h_txtrPtr;
    std::vector<const char *> h_paths;

    int appendTexture(const char *path);

    // Device memory
    Flt4 *d_txtrFlat;
    TxtrPtr *d_txtrPtr;

    int txtrSize = 0;
    int txtrCount = 0;

    void freeDevice();
    void toDevice();
};

#endif