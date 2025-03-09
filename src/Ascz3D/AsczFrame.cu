#include <AsczFrame.cuh>
#include <AzDevMath.cuh>
#include <iostream>
#define _GAMMA 1.0f/2.2f

__global__ void initRandState(curandState *state, int width, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) curand_init(1234, idx, 0, &state[idx]);
}

__global__ void toDrawBuffer(float *fx, float *fy, float *fz, unsigned int *draw, int width, int size, bool toneMap) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size) return;

    float r = AzDevMath::ACESFilm(powf(fx[i], _GAMMA)) * toneMap + fx[i] * !toneMap;
    float g = AzDevMath::ACESFilm(powf(fy[i], _GAMMA)) * toneMap + fy[i] * !toneMap;
    float b = AzDevMath::ACESFilm(powf(fz[i], _GAMMA)) * toneMap + fz[i] * !toneMap;

    draw[i] = (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255);
}


AsczFrame::AsczFrame(int w, int h) : width(w), height(h), size(w * h) {
    blockCount = (size + blockSize - 1) / blockSize;
    // Buffers initialization

    cudaMalloc(&d_fx0, size * sizeof(float));
    cudaMalloc(&d_fy0, size * sizeof(float));
    cudaMalloc(&d_fz0, size * sizeof(float));

    cudaMalloc(&d_fx1, size * sizeof(float));
    cudaMalloc(&d_fy1, size * sizeof(float));
    cudaMalloc(&d_fz1, size * sizeof(float));

    cudaMalloc(&d_fx2, size * sizeof(float));
    cudaMalloc(&d_fy2, size * sizeof(float));
    cudaMalloc(&d_fz2, size * sizeof(float));

    cudaMalloc(&d_draw, size * sizeof(unsigned int));
    h_draw = new unsigned int[size];

    // Random state initialization
    cudaMalloc(&d_rand, size * sizeof(curandState));
    initRandState<<<blockCount, blockSize>>>(d_rand, width, size);
}

AsczFrame::~AsczFrame() {
    cudaFree(d_fx0); cudaFree(d_fy0); cudaFree(d_fz0);
    cudaFree(d_fx1); cudaFree(d_fy1); cudaFree(d_fz1);
    cudaFree(d_fx2); cudaFree(d_fy2); cudaFree(d_fz2);

    cudaFree(d_draw); delete[] h_draw;
}

void AsczFrame::toDraw0(bool toneMap) {
    toDrawBuffer<<<blockCount, blockSize>>>(d_fx0, d_fy0, d_fz0, d_draw, width, size, toneMap);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}
void AsczFrame::toDraw1(bool toneMap) {
    toDrawBuffer<<<blockCount, blockSize>>>(d_fx1, d_fy1, d_fz1, d_draw, width, size, toneMap);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}
void AsczFrame::toDraw2(bool toneMap) {
    toDrawBuffer<<<blockCount, blockSize>>>(d_fx2, d_fy2, d_fz2, d_draw, width, size, toneMap);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}