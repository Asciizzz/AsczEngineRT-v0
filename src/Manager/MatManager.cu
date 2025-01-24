#include <MatManager.cuh>
#include <cuda_runtime.h>

int MatManager::appendMaterial(Material mat) {
    h_mats.push_back(mat);
    return matsNum++;
}

void MatManager::freeDevice() {
    if (d_mats) {
        cudaFree(d_mats);
        d_mats = nullptr;
    }
}

void MatManager::hostToDevice() {
    freeDevice();
    if (matsNum) {
        cudaMalloc(&d_mats, matsNum * sizeof(Material));
        cudaMemcpy(d_mats, h_mats.data(), matsNum * sizeof(Material), cudaMemcpyHostToDevice);
    }
}