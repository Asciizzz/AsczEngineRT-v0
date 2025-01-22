#ifndef TXTRMANAGER_CUH
#define TXTRMANAGER_CUH

#include <cuda_runtime.h>
#include <SFML/Graphics.hpp>
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

    void appendTexture(const char *path) {
        sf::Image img;
        img.loadFromFile(path);

        int w = img.getSize().x;
        int h = img.getSize().y;

        TxtrPtr txtrPtr = {w, h, txtrSize};
        h_txtrPtr.push_back(txtrPtr);
        h_paths.push_back(path);

        txtrSize += w * h;
        txtrCount++;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                sf::Color c = img.getPixel(x, h - y - 1);
                h_txtrFlat.push_back(Vec3f(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f));
            }
        }
    }

    // Device memory
    Vec3f *d_txtrFlat;
    TxtrPtr *d_txtrPtr;

    int txtrSize = 0;
    int txtrCount = 0;

    void freeDevice() {
        if (txtrSize == 0) return;

        cudaFree(d_txtrFlat);
        cudaFree(d_txtrPtr);
    }

    void hostToDevice() {
        freeDevice();

        cudaMalloc(&d_txtrFlat, txtrSize * sizeof(Vec3f));
        cudaMalloc(&d_txtrPtr, txtrCount * sizeof(TxtrPtr));

        cudaMemcpy(d_txtrFlat, h_txtrFlat.data(), txtrSize * sizeof(Vec3f), cudaMemcpyHostToDevice);
        cudaMemcpy(d_txtrPtr, h_txtrPtr.data(), txtrCount * sizeof(TxtrPtr), cudaMemcpyHostToDevice);
    }
};

#endif