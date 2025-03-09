#ifndef TODEVICE_CUH
#define TODEVICE_CUH

#include <Vector.cuh>

class ToDevice {
public:
    static void F(float *d, const float *h, int n) {
        cudaMalloc(&d, n * sizeof(float));
        cudaMemcpy(d, h, n * sizeof(float), cudaMemcpyHostToDevice);
    }
    static void F(std::vector<float> &h, float *&d, int size=-1) {
        if (size == -1) size = h.size();
        cudaMalloc(&d, size * sizeof(float));
        cudaMemcpy(d, h.data(), size * sizeof(float), cudaMemcpyHostToDevice);
    }

    static void I(int *d, const int *h, int n) {
        cudaMalloc(&d, n * sizeof(int));
        cudaMemcpy(d, h, n * sizeof(int), cudaMemcpyHostToDevice);
    }
    static void I(std::vector<int> &h, int *&d, int size=-1) {
        if (size == -1) size = h.size();
        cudaMalloc(&d, size * sizeof(int));
        cudaMemcpy(d, h.data(), size * sizeof(int), cudaMemcpyHostToDevice);
    }

    static void F3(Flt3 *d, const Flt3 *h, int n) {
        cudaMalloc(&d, n * sizeof(Flt3));
        cudaMemcpy(d, h, n * sizeof(Flt3), cudaMemcpyHostToDevice);
    }
    static void F3(Vec3f &h, Flt3 *&d, int size=-1) {
        if (size == -1) size = h.size();
        cudaMalloc(&d, size * sizeof(Flt3));
        cudaMemcpy(d, h.data(), size * sizeof(Flt3), cudaMemcpyHostToDevice);
    }
};

#endif