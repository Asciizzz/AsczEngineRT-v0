#ifndef PATHTRACESTD_CUH
#define PATHTRACESTD_CUH

#include <AsczCam.cuh>

#include <curand_kernel.h>

__global__ void pathtraceSTDKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,

    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,

    // AzMtl *mats, int *lsrc, int lNum,
    
    float *TX_r, float *TX_g, float *TX_b, float *TX_a,
    int *TX_w, int *TX_h, int *TX_off,
    
    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *BV_pl, int *BV_pr, bool *BV_lf, int *BV_fi,

    curandState *rnd
);

#endif