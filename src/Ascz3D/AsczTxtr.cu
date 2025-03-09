#define STB_IMAGE_IMPLEMENTATION

#include <AsczTxtr.cuh>
#include <stb_image.h>
#include <cuda_runtime.h>

AsczTxtr::~AsczTxtr() {
    cudaFree(d_tr);
    cudaFree(d_tg);
    cudaFree(d_tb);
    cudaFree(d_ta);

    cudaFree(d_tw);
    cudaFree(d_th);
    cudaFree(d_toff);
}

int AsczTxtr::appendTexture(const char *path) {
    int w, h, n;
    unsigned char *data = stbi_load(path, &w, &h, &n, 4);

    if (data == nullptr) return -1;

    h_tw.push_back(w);
    h_th.push_back(h);
    h_toff.push_back(size);

    for (int y = h - 1; y > -1; y--) {
        for (int x = 0; x < w; x++) {
            int i = (y * w + x) * 4;

            h_tr.push_back(data[i + 0] / 255.0f);
            h_tg.push_back(data[i + 1] / 255.0f);
            h_tb.push_back(data[i + 2] / 255.0f);
            h_ta.push_back(data[i + 3] / 255.0f);
        }
    }

    size += w * h;

    return count++;
}

void AsczTxtr::toDevice() {
    cudaMalloc(&d_tr, size * sizeof(float));
    cudaMalloc(&d_tg, size * sizeof(float));
    cudaMalloc(&d_tb, size * sizeof(float));
    cudaMalloc(&d_ta, size * sizeof(float));

    cudaMemcpy(d_tr, h_tr.data(), size * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_tg, h_tg.data(), size * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_tb, h_tb.data(), size * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ta, h_ta.data(), size * sizeof(float), cudaMemcpyHostToDevice);

    cudaMalloc(&d_tw, count * sizeof(int));
    cudaMalloc(&d_th, count * sizeof(int));
    cudaMalloc(&d_toff, count * sizeof(int));

    cudaMemcpy(d_tw, h_tw.data(), count * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_th, h_th.data(), count * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_toff, h_toff.data(), count * sizeof(int), cudaMemcpyHostToDevice);
}