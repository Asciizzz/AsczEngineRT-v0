#include <AsczLight.cuh>

void AsczLight::appendLight(LightSrc light) {
    h_lSrc.push_back(light);
}

void AsczLight::toDevice() {
    num = h_lSrc.size();
    cudaMalloc(&d_lSrc, num * sizeof(LightSrc));
    cudaMemcpy(d_lSrc, h_lSrc.data(), num * sizeof(LightSrc), cudaMemcpyHostToDevice);
}

void AsczLight::freeDevice() {
    if (num == 0) return;
    cudaFree(d_lSrc);
}