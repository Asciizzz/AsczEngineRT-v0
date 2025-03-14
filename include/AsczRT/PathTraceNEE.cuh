#ifndef PATHTRACENEE_CUH
#define PATHTRACENEE_CUH

#include <AsczMat.cuh>
#include <AsczCam.cuh>

#include <curand_kernel.h>

__global__ void pathtraceNEEKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,
    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,
    AzMtl *mats, int *lsrc, int lNum,
    float *TX_r, float *TX_g, float *TX_b, float *TX_a, int *TX_w, int *TX_h, int *TX_off,
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    curandState *rnd
);

#endif