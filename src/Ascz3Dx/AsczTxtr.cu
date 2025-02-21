#define STB_IMAGE_IMPLEMENTATION

#include <AsczTxtr.cuh>
#include <stb_image.h>
#include <cuda_runtime.h>
#include <iostream>

int AsczTxtr::appendTexture(const char *path) {
    int w, h, n;
    unsigned char *data = stbi_load(path, &w, &h, &n, 4);

    if (data == nullptr) return -1;

    h_paths.push_back(path);
    h_txtrPtr.push_back({w, h, txtrSize});
    txtrCount++;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int i = ((h - y - 1) * w + x) * 4;
            float r = data[i + 0] / 255.0f;
            float g = data[i + 1] / 255.0f;
            float b = data[i + 2] / 255.0f;
            float a = data[i + 3] / 255.0f;

            h_txtrFlat.push_back({r, g, b, a});
        }
    }

    txtrSize += w * h;

    return txtrCount - 1;
}

void AsczTxtr::freeDevice() {
    if (txtrSize == 0) return;

    cudaFree(d_txtrFlat);
    cudaFree(d_txtrPtr);
}

void AsczTxtr::toDevice() {
    freeDevice();

    cudaMalloc(&d_txtrFlat, txtrSize * sizeof(Flt4));
    cudaMalloc(&d_txtrPtr, txtrCount * sizeof(TxtrPtr));

    cudaMemcpy(d_txtrFlat, h_txtrFlat.data(), txtrSize * sizeof(Flt4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_txtrPtr, h_txtrPtr.data(), txtrCount * sizeof(TxtrPtr), cudaMemcpyHostToDevice);
}