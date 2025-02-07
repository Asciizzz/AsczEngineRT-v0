#include <AsczMat.cuh>
#include <cuda_runtime.h>

int AsczMat::appendMaterial(Material mat) {
    h_mats.push_back(mat);
    return matsNum++;
}

void AsczMat::freeDevice() {
    if (d_mats) {
        cudaFree(d_mats);
        d_mats = nullptr;
    }
}

void AsczMat::toDevice() {
    freeDevice();
    if (matsNum) {
        cudaMalloc(&d_mats, matsNum * sizeof(Material));
        cudaMemcpy(d_mats, h_mats.data(), matsNum * sizeof(Material), cudaMemcpyHostToDevice);
    }
}