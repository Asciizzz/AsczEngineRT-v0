#ifndef TXTRMANAGER_CUH
#define TXTRMANAGER_CUH

#include <Vector.cuh>

struct TxtrPtr {
    int w, h, off;
};

class TxtrManager {
public:
    // Host memory
    std::vector<Vec3f> h_txtrFlat;
    std::vector<TxtrPtr> h_txtrPtr;
    std::vector<const char *> h_paths;

    int appendTexture(const char *path);

    // Device memory
    Vec3f *d_txtrFlat;
    TxtrPtr *d_txtrPtr;

    int txtrSize = 0;
    int txtrCount = 0;

    void freeDevice();
    void hostToDevice();
};

#endif