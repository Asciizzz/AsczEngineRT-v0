#include <AsczMat.cuh>
#include <cuda_runtime.h>

int AsczMat::appendMaterial(Material mtl) {
    h_mtls.push_back(mtl);
    return mtlsNum++;
}

void AsczMat::freeDevice() {
    if (d_mtls) {
        cudaFree(d_mtls);
        d_mtls = nullptr;
    }
}

void AsczMat::toDevice() {
    freeDevice();
    if (mtlsNum) {
        cudaMalloc(&d_mtls, mtlsNum * sizeof(Material));
        cudaMemcpy(d_mtls, h_mtls.data(), mtlsNum * sizeof(Material), cudaMemcpyHostToDevice);
    }
}