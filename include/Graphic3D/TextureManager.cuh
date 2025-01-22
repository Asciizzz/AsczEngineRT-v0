#ifndef TEXTUREMANAGER_CUH
#define TEXTUREMANAGER_CUH

#include <cuda_runtime.h>
#include <SFML/Graphics.hpp>
#include <Vector.cuh>

class TextureManager {
public:
    // Host memory
    std::vector<Vec3f> h_txtr;
    std::vector<int> h_w, h_h;
    std::vector<int> h_off;
    std::vector<const char *> h_paths;

    void appendTexture(const char *path) {
        sf::Image img;
        img.loadFromFile(path);

        int w = img.getSize().x;
        int h = img.getSize().y;

        h_w.push_back(w);
        h_h.push_back(h);
        h_off.push_back(txtrSize);
        h_paths.push_back(path);

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                sf::Color c = img.getPixel(x, h - y - 1);
                h_txtr.push_back(Vec3f(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f));
            }
        }

        txtrSize += w * h;
        txtrCount++;
    }

    // Device memory
    Vec3f *d_txtr; // A flat array of all textures
    int *d_w, *d_h, *d_off; // Pointers to that texture

    int txtrSize = 0;
    int txtrCount = 0;

    void freeDevice() {
        if (txtrSize == 0) return;

        cudaFree(d_txtr);
        cudaFree(d_w);
        cudaFree(d_h);
        cudaFree(d_off);
    }

    void hostToDevice() {
        freeDevice();

        cudaMalloc(&d_txtr, txtrSize * sizeof(Vec3f));
        cudaMalloc(&d_w, txtrCount * sizeof(int));
        cudaMalloc(&d_h, txtrCount * sizeof(int));
        cudaMalloc(&d_off, txtrCount * sizeof(int));

        cudaMemcpy(d_txtr, h_txtr.data(), txtrSize * sizeof(Vec3f), cudaMemcpyHostToDevice);
        cudaMemcpy(d_w, h_w.data(), txtrCount * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_h, h_h.data(), txtrCount * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_off, h_off.data(), txtrCount * sizeof(int), cudaMemcpyHostToDevice);
    }
};

#endif