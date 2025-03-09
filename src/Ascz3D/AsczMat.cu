#include <AsczMat.cuh>
#include <cuda_runtime.h>

AsczMat::~AsczMat() {
    if (d_mtls) {
        cudaFree(d_mtls);
        d_mtls = nullptr;
    }
}

int AsczMat::append(AzMtl mtl, std::wstring name, std::wstring path) {
    h_mtls.push_back(mtl);
    names.push_back(name);
    paths.push_back(path);
    return mtlsNum++;
}

void AsczMat::toDevice() {
    if (mtlsNum) {
        cudaMalloc(&d_mtls, mtlsNum * sizeof(AzMtl));
        cudaMemcpy(d_mtls, h_mtls.data(), mtlsNum * sizeof(AzMtl), cudaMemcpyHostToDevice);
    }
}