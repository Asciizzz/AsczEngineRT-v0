#include <cuda_runtime.h>
#include <iostream>
#include <chrono>
#include <cmath>

// Constants
const int WIDTH = 1920;
const int HEIGHT = 1080;
const int NUM_TRIANGLES = 1000000;

// Device memory pointers
__device__ int d_x[NUM_TRIANGLES * 3];
__device__ int d_y[NUM_TRIANGLES * 3];
__device__ float d_z[NUM_TRIANGLES * 3];
__device__ float d_depthBuffer[WIDTH * HEIGHT];
__device__ int d_indexBuffer[WIDTH * HEIGHT];
__device__ int d_prevIndexBuffer[WIDTH * HEIGHT];

// Helper: Calculate barycentric coordinates
__device__ bool isInsideTriangle(int x, int y, int x0, int y0, int x1, int y1, int x2, int y2) {
    int denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    float a = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / (float)denom;
    float b = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / (float)denom;
    float c = 1.0f - a - b;
    return a >= 0 && b >= 0 && c >= 0;
}

// Rasterization kernel
__global__ void rasterizeKernel() {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int numPixels = WIDTH * HEIGHT;
    if (idx >= numPixels) return;

    int px = idx % WIDTH;
    int py = idx / WIDTH;

    for (int i = 0; i < NUM_TRIANGLES; i++) {
        int x0 = d_x[i * 3], y0 = d_y[i * 3];
        int x1 = d_x[i * 3 + 1], y1 = d_y[i * 3 + 1];
        int x2 = d_x[i * 3 + 2], y2 = d_y[i * 3 + 2];
        float z0 = d_z[i * 3], z1 = d_z[i * 3 + 1], z2 = d_z[i * 3 + 2];

        if (isInsideTriangle(px, py, x0, y0, x1, y1, x2, y2)) {
            float depth = (z0 + z1 + z2) / 3.0f;
            if (depth < d_depthBuffer[idx]) {
                d_depthBuffer[idx] = depth;
                d_indexBuffer[idx] = i;
            }
        }
    }
}

// Race condition validation kernel
__global__ void validateKernel(bool* result) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= WIDTH * HEIGHT) return;

    if (d_indexBuffer[idx] != d_prevIndexBuffer[idx]) {
        *result = false;
    }
}

// Main function
int main() {
    // Host buffers
    int h_x[NUM_TRIANGLES * 3];
    int h_y[NUM_TRIANGLES * 3];
    float h_z[NUM_TRIANGLES * 3];

    // Generate random test data
    for (int i = 0; i < NUM_TRIANGLES * 3; i++) {
        h_x[i] = rand() % WIDTH;
        h_y[i] = rand() % HEIGHT;
        h_z[i] = static_cast<float>(rand()) / RAND_MAX;
    }

    // Copy to device
    cudaMemcpyToSymbol(d_x, h_x, sizeof(h_x));
    cudaMemcpyToSymbol(d_y, h_y, sizeof(h_y));
    cudaMemcpyToSymbol(d_z, h_z, sizeof(h_z));

    // Initialize buffers
    cudaMemset(d_depthBuffer, 0x7F, sizeof(float) * WIDTH * HEIGHT); // FLT_MAX
    cudaMemset(d_indexBuffer, -1, sizeof(int) * WIDTH * HEIGHT);
    cudaMemset(d_prevIndexBuffer, -1, sizeof(int) * WIDTH * HEIGHT);

    // Timing variables
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        // Rasterize
        dim3 blockDim(256);
        dim3 gridDim((WIDTH * HEIGHT + blockDim.x - 1) / blockDim.x);
        rasterizeKernel<<<gridDim, blockDim>>>();

        // Validate
        bool h_result = true;
        bool* d_result;
        cudaMalloc(&d_result, sizeof(bool));
        cudaMemcpy(d_result, &h_result, sizeof(bool), cudaMemcpyHostToDevice);

        validateKernel<<<gridDim, blockDim>>>(d_result);

        cudaMemcpy(&h_result, d_result, sizeof(bool), cudaMemcpyDeviceToHost);
        cudaFree(d_result);

        // Log time
        auto end = std::chrono::high_resolution_clock::now();
        double tte = std::chrono::duration<double, std::milli>(end - start).count();
        start = end;

        std::cout << tte << "ms " << (h_result ? "true" : "false") << std::endl;

        // Update previous buffer
        cudaMemcpyToSymbol(d_prevIndexBuffer, d_indexBuffer, sizeof(int) * WIDTH * HEIGHT);
    }

    return 0;
}
