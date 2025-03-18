#ifndef RAYCAST_CUH
#define RAYCAST_CUH

#include <AsczCam.cuh>

__global__ void raycastKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,

    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,

    float *MT_Alb_r, float *MT_Alb_g, float *MT_Alb_b, int *MT_AlbMap,

    float *TX_r, float *TX_g, float *TX_b, float *TX_a,
    int *TX_w, int *TX_h, int *TX_off,

    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *BV_pl, int *BV_pr, bool *BV_lf, int *BV_fi,

    bool fakeShading,
    float *frmdepth, int *frmmat
);

#endif