#include <AsczMtl.cuh>
#include <cuda_runtime.h>

int AsczMtl::appendMaterial(Material mtl) {
    h_mtls.push_back(mtl);
    return mtlsNum++;
}

void AsczMtl::freeDevice() {
    if (d_mtls) {
        cudaFree(d_mtls);
        d_mtls = nullptr;
    }
}

void AsczMtl::toDevice() {
    freeDevice();
    if (mtlsNum) {
        cudaMalloc(&d_mtls, mtlsNum * sizeof(Material));
        cudaMemcpy(d_mtls, h_mtls.data(), mtlsNum * sizeof(Material), cudaMemcpyHostToDevice);
    }
}