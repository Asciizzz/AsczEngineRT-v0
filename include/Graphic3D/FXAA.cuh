#ifndef FXAA_CUH
#define FXAA_CUH

#include <Vector.cuh>

__global__ void calcLuminance(float *lumi, Flt3 *frmbuffer, int frmW, int frmH) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= frmW * frmH) return;

    Flt3 color = frmbuffer[idx];
    lumi[idx] = 0.299f * color.x + 0.587f * color.y + 0.114f * color.z;
}

__global__ void edgeMask(bool *edge, float *lumi, int frmW, int frmH) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= frmW * frmH) return;

    int x = idx % frmW, y = idx / frmW;

    if (x == 0 || y == 0 || x == frmW - 1 || y == frmH - 1) {
        edge[idx] = false;
        return;
    }

    float lumiL = lumi[idx - 1];
    float lumiR = lumi[idx + 1];
    float lumiT = lumi[idx - frmW];
    float lumiB = lumi[idx + frmW];

    float gradX = abs(lumiL - lumiR) + abs(lumiT - lumiB);
    float gradY = abs(lumiT - lumiB) + abs(lumiL - lumiR);

    edge[idx] = gradX + gradY >= 0.1f;
}

__global__ void applyFXAAtoBuffer(float *lumi, bool *edge, Flt3 *frmbuffer1, Flt3 *frmbuffer2, int frmW, int frmH) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= frmW * frmH) return;

    if (!edge[idx]) {
        frmbuffer2[idx] = frmbuffer1[idx];
        return;
    }

    Flt3 colrL = frmbuffer1[idx - 1];
    Flt3 colrR = frmbuffer1[idx + 1];
    Flt3 colrT = frmbuffer1[idx - frmW];
    Flt3 colrB = frmbuffer1[idx + frmW];
    Flt3 colrLT = frmbuffer1[idx - frmW - 1];
    Flt3 colrRT = frmbuffer1[idx - frmW + 1];
    Flt3 colrLB = frmbuffer1[idx + frmW - 1];
    Flt3 colrRB = frmbuffer1[idx + frmW + 1];

    float lumiC = lumi[idx];
    float lumiL = lumi[idx - 1];
    float lumiR = lumi[idx + 1];
    float lumiT = lumi[idx - frmW];
    float lumiB = lumi[idx + frmW];
    float lumiLT = lumi[idx - frmW - 1];
    float lumiRT = lumi[idx - frmW + 1];
    float lumiLB = lumi[idx + frmW - 1];
    float lumiRB = lumi[idx + frmW + 1];

    float wL = 1.0f / (1.0f + abs(lumiC - lumiL));
    float wR = 1.0f / (1.0f + abs(lumiC - lumiR));
    float wT = 1.0f / (1.0f + abs(lumiC - lumiT));
    float wB = 1.0f / (1.0f + abs(lumiC - lumiB));
    float wLT = 1.0f / (1.0f + abs(lumiC - lumiLT));
    float wRT = 1.0f / (1.0f + abs(lumiC - lumiRT));
    float wLB = 1.0f / (1.0f + abs(lumiC - lumiLB));
    float wRB = 1.0f / (1.0f + abs(lumiC - lumiRB));

    float wSum = wL + wR + wT + wB + wLT + wRT + wLB + wRB;
    wL /= wSum; wR /= wSum; wT /= wSum; wB /= wSum;
    wLT /= wSum; wRT /= wSum; wLB /= wSum; wRB /= wSum;

    frmbuffer2[idx] = (
        colrL * wL + colrR * wR + colrT * wT + colrB * wB +
        colrLT * wLT + colrRT * wRT + colrLB * wLB + colrRB * wRB
    );
}

#endif