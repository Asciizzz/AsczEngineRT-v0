#include <TxtrManager.cuh>
#include <SFML/Graphics.hpp>
#include <cuda_runtime.h>

int TxtrManager::appendTexture(const char *path) {
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
            h_txtrFlat.push_back(Vec3f(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f));
        }
    }

    // Return the index of the texture
    return txtrCount++;
}

void TxtrManager::freeDevice() {
    if (txtrSize == 0) return;

    cudaFree(d_txtrFlat);
    cudaFree(d_txtrPtr);
}

void TxtrManager::toDevice() {
    freeDevice();

    cudaMalloc(&d_txtrFlat, txtrSize * sizeof(Vec3f));
    cudaMalloc(&d_txtrPtr, txtrCount * sizeof(TxtrPtr));

    cudaMemcpy(d_txtrFlat, h_txtrFlat.data(), txtrSize * sizeof(Vec3f), cudaMemcpyHostToDevice);
    cudaMemcpy(d_txtrPtr, h_txtrPtr.data(), txtrCount * sizeof(TxtrPtr), cudaMemcpyHostToDevice);
}