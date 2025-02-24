#define STB_IMAGE_IMPLEMENTATION

#include <AsczTxtr.cuh>
#include <stb_image.h>
#include <cuda_runtime.h>
#include <iostream>

int AsczTxtr::appendTexture(const char *path) {
    int w, h, n;
    unsigned char *data = stbi_load(path, &w, &h, &n, 4);

    if (data == nullptr) return -1;

    h_ptr.push_back({w, h, size});

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int i = ((h - y - 1) * w + x) * 4;
            float r = data[i + 0] / 255.0f;
            float g = data[i + 1] / 255.0f;
            float b = data[i + 2] / 255.0f;
            float a = data[i + 3] / 255.0f;

            h_flat.push_back({r, g, b, a});
        }
    }

    size += w * h;

    return count++;
}

void AsczTxtr::freeDevice() {
    if (size == 0) return;

    cudaFree(d_flat);
    cudaFree(d_ptr);
}

void AsczTxtr::toDevice() {
    freeDevice();

    cudaMalloc(&d_flat, size * sizeof(Flt4));
    cudaMalloc(&d_ptr, count * sizeof(TxtrPtr));

    cudaMemcpy(d_flat, h_flat.data(), size * sizeof(Flt4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ptr, h_ptr.data(), count * sizeof(TxtrPtr), cudaMemcpyHostToDevice);
}