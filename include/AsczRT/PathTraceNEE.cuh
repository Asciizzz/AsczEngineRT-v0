#ifndef PATHTRACENEE_CUH
#define PATHTRACENEE_CUH

#include <AsczCam.cuh>

#include <curand_kernel.h>

__global__ void pathtraceNEEKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,

    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,
    int *MS_lsrc, int MS_lnum,

    float *MT_alb_r, float *MT_alb_g, float *MT_alb_b, int *MT_alb_map,
    float *MT_rough, float *MT_metal, float *MT_tr, float *MT_ior,
    float *MT_ems_r, float *MT_ems_g, float *MT_ems_b, float *MT_ems_i,

    float *TX_r, float *TX_g, float *TX_b, float *TX_a,
    int *TX_w, int *TX_h, int *TX_off,

    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *BV_pl, int *BV_pr, bool *BV_lf, int *BV_fi,

    // curandState *rnd
    uint32_t seed
);

#endif