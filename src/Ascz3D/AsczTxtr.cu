#include <AsczTxtr.cuh>
#include <SFML/Graphics.hpp>
#include <cuda_runtime.h>

#include <iostream>

int AsczTxtr::appendTexture(const char *path) {
    sf::Image img;
    img.loadFromFile(path);

    int w = img.getSize().x;
    int h = img.getSize().y;

    TxtrPtr txtrPtr = {w, h, txtrSize};
    h_txtrPtr.push_back(txtrPtr);
    h_paths.push_back(path);

    txtrSize += w * h;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            sf::Color c = img.getPixel(x, h - y - 1);

            float r = c.r / 255.0f;
            float g = c.g / 255.0f;
            float b = c.b / 255.0f;
            float a = c.a / 255.0f;

            h_txtrFlat.push_back(Flt4{r, g, b, a});
        }
    }

    // Return the index of the texture
    return txtrCount++;
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