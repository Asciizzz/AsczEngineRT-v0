#include <AsczFrame.cuh>
#include <AzDevMath.cuh>
#include <iostream>
#define _GAMMA 1.0f/2.2f

__global__ void initRandState(curandState *state, int width, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) curand_init(1234, idx, 0, &state[idx]);
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

    cudaMalloc(&d_rand, size * sizeof(curandState));
    initRandState<<<blockCount, blockSize>>>(d_rand, width, size);

    cudaMalloc(&d_depth, size * sizeof(float));
    h_depth = new float[size];

    cudaMalloc(&d_mat, size * sizeof(int));
    h_mat = new int[size];
}

AsczFrame::~AsczFrame() {
    cudaFree(d_fx0); cudaFree(d_fy0); cudaFree(d_fz0);
    cudaFree(d_fx1); cudaFree(d_fy1); cudaFree(d_fz1);
    cudaFree(d_fx2); cudaFree(d_fy2); cudaFree(d_fz2);
    
    cudaFree(d_draw); delete[] h_draw;

    cudaFree(d_rand);

    cudaFree(d_depth); delete[] h_depth;
    cudaFree(d_mat); delete[] h_mat;
}



__global__ void toDrawBuffer(float *fx, float *fy, float *fz, unsigned int *draw, int width, int size, bool toneMap, bool crosshair=false) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size) return;

    int x = i % width;
    int y = i / width;
    int height = size / width;
    int h_width = width / 2;
    int h_height = height / 2;
    int cr_thick = 1;
    int cr_length = 5;

    float r = AzDevMath::ACESFilm(powf(fx[i], _GAMMA)) * toneMap + fx[i] * !toneMap;
    float g = AzDevMath::ACESFilm(powf(fy[i], _GAMMA)) * toneMap + fy[i] * !toneMap;
    float b = AzDevMath::ACESFilm(powf(fz[i], _GAMMA)) * toneMap + fz[i] * !toneMap;

    // Draw a crosshair
    int x_sub_hw = x - h_width;
    int y_sub_hh = y - h_height;

    bool isCrosshair = crosshair &
        (x_sub_hw >= -cr_thick & x_sub_hw <= cr_thick & y_sub_hh >= -cr_length & y_sub_hh <= cr_length) |
        (y_sub_hh >= -cr_thick & y_sub_hh <= cr_thick & x_sub_hw >= -cr_length & x_sub_hw <= cr_length);

    int ir = r * 255;
    int ig = g * 255;
    int ib = b * 255;

    // This to ensure the cross hair will never share the same color as the background
    int truer = ir * !isCrosshair + ((ir + 128) % 255) * isCrosshair;
    int trueg = ig * !isCrosshair + ((ig + 128) % 255) * isCrosshair;
    int trueb = ib * !isCrosshair + ((ib + 128) % 255) * isCrosshair;

    draw[i] = (truer << 16) | (trueg << 8) | trueb;
}

void AsczFrame::toDraw0(bool toneMap, bool crosshair) {
    toDrawBuffer<<<blockCount, blockSize>>>(d_fx0, d_fy0, d_fz0, d_draw, width, size, toneMap, crosshair);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}

void AsczFrame::toDraw1(bool toneMap, bool crosshair) {
    toDrawBuffer<<<blockCount, blockSize>>>(d_fx1, d_fy1, d_fz1, d_draw, width, size, toneMap, crosshair);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}


__global__ void toDrawAccumulatedBuffer(float *fx, float *fy, float *fz, unsigned int *draw, int width, int size, int acc, bool toneMap) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size) return;

    float rdiv = fx[i] / acc;
    float gdiv = fy[i] / acc;
    float bdiv = fz[i] / acc;

    float r = AzDevMath::ACESFilm(powf(rdiv, _GAMMA)) * toneMap + rdiv * !toneMap;
    float g = AzDevMath::ACESFilm(powf(gdiv, _GAMMA)) * toneMap + gdiv * !toneMap;
    float b = AzDevMath::ACESFilm(powf(bdiv, _GAMMA)) * toneMap + bdiv * !toneMap;

    draw[i] = (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255);
}

void AsczFrame::toDraw2(bool toneMap) {
    toDrawAccumulatedBuffer<<<blockCount, blockSize>>>(d_fx2, d_fy2, d_fz2, d_draw, width, size, f_acc, toneMap);
    cudaMemcpy(h_draw, d_draw, size * sizeof(unsigned int), cudaMemcpyDeviceToHost);
}



__global__ void addKernel(float *fx, float *fy, float *fz, float *fx2, float *fy2, float *fz2, int size) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size) return;

    fx[i] += fx2[i];
    fy[i] += fy2[i];
    fz[i] += fz2[i];
}

void AsczFrame::add0() {
    addKernel<<<blockCount, blockSize>>>(d_fx2, d_fy2, d_fz2, d_fx0, d_fy0, d_fz0, size);
    f_acc++;
}
void AsczFrame::add1() {
    addKernel<<<blockCount, blockSize>>>(d_fx2, d_fy2, d_fz2, d_fx1, d_fy1, d_fz1, size);
    f_acc++;
}


__global__ void resetKernel(float *fx, float *fy, float *fz, int size) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size) {
        fx[i] = 0.0f; fy[i] = 0.0f; fz[i] = 0.0f;
    }
}

void AsczFrame::reset2() {
    resetKernel<<<blockCount, blockSize>>>(d_fx2, d_fy2, d_fz2, size);
    f_acc = 0;
}


__global__ void bilateralFilter(
    float *fx0, float *fy0, float *fz0,  // Input frame
    float *fx1, float *fy1, float *fz1,  // Output frame
    float *depth,                         // Depth buffer
    int width, int height,                // Image dimensions
    float sigmaSpatial, float sigmaRange, // Filter parameters
    int radius                            // Filter radius
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int x = idx % width, y = idx / width;

    if (x >= width || y >= height) return;

    float centerDepth = depth[idx];
    float centerR = fx0[idx];
    float centerG = fy0[idx];
    float centerB = fz0[idx];

    float sumR = 0.0f, sumG = 0.0f, sumB = 0.0f;
    float totalWeight = 0.0f;

    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            int nIdx = ny * width + nx;
            float neighborDepth = depth[nIdx];
            float neighborR = fx0[nIdx];
            float neighborG = fy0[nIdx];
            float neighborB = fz0[nIdx];

            // Compute spatial weight (Gaussian blur based on distance)
            float spatialWeight = expf(-(dx * dx + dy * dy) / (2.0f * sigmaSpatial * sigmaSpatial));

            // Compute range weight (Gaussian depth similarity)
            float rangeWeight = expf(-((neighborDepth - centerDepth) * (neighborDepth - centerDepth)) / (2.0f * sigmaRange * sigmaRange));

            // Final weight
            float weight = spatialWeight * rangeWeight;
            totalWeight += weight;

            // Apply weighted sum
            sumR += neighborR * weight;
            sumG += neighborG * weight;
            sumB += neighborB * weight;
        }
    }

    // Normalize and write back
    fx1[idx] = sumR / totalWeight;
    fy1[idx] = sumG / totalWeight;
    fz1[idx] = sumB / totalWeight;
}

void AsczFrame::biliFilter0() {
    bilateralFilter<<<blockCount, blockSize>>>(
        d_fx0, d_fy0, d_fz0,
        d_fx1, d_fy1, d_fz1,
        d_depth, width, height,
        1.0f, 0.1f, 3
    );
}

void AsczFrame::biliFilter1() {
    bilateralFilter<<<blockCount, blockSize>>>(
        d_fx1, d_fy1, d_fz1,
        d_fx0, d_fy0, d_fz0,
        d_depth, width, height,
        1.0f, 0.1f, 3
    );
}