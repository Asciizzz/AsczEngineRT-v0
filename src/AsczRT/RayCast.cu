#include <RayCast.cuh>

__global__ void raycastKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,

    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,

    AzMtl *mats,

    float *TX_r, float *TX_g, float *TX_b, float *TX_a,
    int *TX_w, int *TX_h, int *TX_off,

    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *BV_pl, int *BV_pr, bool *BV_lf, int *BV_fi,

    bool fakeShading,
    float *frmdepth, int *frmmat
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmw * frmh) return;

    Ray ray = camera.castRay(tIdx % frmw, tIdx / frmw, frmw, frmh);

    // Hit info
    int H_Idx = -1;
    float H_t = 1e9f;
    float H_u = 0.0f;
    float H_v = 0.0f;
    float H_w = 0.0f;

    const int MAX_NODES = 32;
    int nstack[MAX_NODES] = { 0 };
    int ns_top = 1;

    while (ns_top > 0) {
        int nidx = nstack[--ns_top];

        // Check if the ray is outside the bounding box
        float t1n = (BV_min_x[nidx] - ray.ox) * ray.rdx;
        float t2n = (BV_max_x[nidx] - ray.ox) * ray.rdx;
        float t3n = (BV_min_y[nidx] - ray.oy) * ray.rdy;
        float t4n = (BV_max_y[nidx] - ray.oy) * ray.rdy;
        float t5n = (BV_min_z[nidx] - ray.oz) * ray.rdz;
        float t6n = (BV_max_z[nidx] - ray.oz) * ray.rdz;

        float tminn1 = fminf(t1n, t2n), tmaxn1 = fmaxf(t1n, t2n);
        float tminn2 = fminf(t3n, t4n), tmaxn2 = fmaxf(t3n, t4n);
        float tminn3 = fminf(t5n, t6n), tmaxn3 = fmaxf(t5n, t6n);

        float tminn = fmaxf(fmaxf(tminn1, tminn2), tminn3);
        float tmaxn = fminf(fminf(tmaxn1, tmaxn2), tmaxn3);

        bool nOut = ray.ox < BV_min_x[nidx] | ray.ox > BV_max_x[nidx] |
                    ray.oy < BV_min_y[nidx] | ray.oy > BV_max_y[nidx] |
                    ray.oz < BV_min_z[nidx] | ray.oz > BV_max_z[nidx];
        bool nMiss = tmaxn < tminn | (tminn < 0 & nOut) | tminn > H_t;

        if (nMiss) continue;

        // If node is not a leaf:
        if (!BV_lf[nidx]) {
            // Find the distance to the left child
            int tcl = BV_pl[nidx];
            float t1l = (BV_min_x[tcl] - ray.ox) * ray.rdx;
            float t2l = (BV_max_x[tcl] - ray.ox) * ray.rdx;
            float t3l = (BV_min_y[tcl] - ray.oy) * ray.rdy;
            float t4l = (BV_max_y[tcl] - ray.oy) * ray.rdy;
            float t5l = (BV_min_z[tcl] - ray.oz) * ray.rdz;
            float t6l = (BV_max_z[tcl] - ray.oz) * ray.rdz;

            float tminl1 = fminf(t1l, t2l), tmaxl1 = fmaxf(t1l, t2l);
            float tminl2 = fminf(t3l, t4l), tmaxl2 = fmaxf(t3l, t4l);
            float tminl3 = fminf(t5l, t6l), tmaxl3 = fmaxf(t5l, t6l);

            float tminl = fmaxf(fmaxf(tminl1, tminl2), tminl3);
            float tmaxl = fminf(fminf(tmaxl1, tmaxl2), tmaxl3);

            bool lOut = ray.ox < BV_min_x[tcl] | ray.ox > BV_max_x[tcl] |
                        ray.oy < BV_min_y[tcl] | ray.oy > BV_max_y[tcl] |
                        ray.oz < BV_min_z[tcl] | ray.oz > BV_max_z[tcl];
            bool lMiss = tmaxl < tminl | tminl < 0;
            float lDist = (-lMiss + tminl * !lMiss) * lOut;

            // Find the distance to the right child
            int tcr = BV_pr[nidx];
            float t1r = (BV_min_x[tcr] - ray.ox) * ray.rdx;
            float t2r = (BV_max_x[tcr] - ray.ox) * ray.rdx;
            float t3r = (BV_min_y[tcr] - ray.oy) * ray.rdy;
            float t4r = (BV_max_y[tcr] - ray.oy) * ray.rdy;
            float t5r = (BV_min_z[tcr] - ray.oz) * ray.rdz;
            float t6r = (BV_max_z[tcr] - ray.oz) * ray.rdz;

            float tminr1 = fminf(t1r, t2r), tmaxr1 = fmaxf(t1r, t2r);
            float tminr2 = fminf(t3r, t4r), tmaxr2 = fmaxf(t3r, t4r);
            float tminr3 = fminf(t5r, t6r), tmaxr3 = fmaxf(t5r, t6r);

            float tminr = fmaxf(fmaxf(tminr1, tminr2), tminr3);
            float tmaxr = fminf(fminf(tmaxr1, tmaxr2), tmaxr3);

            bool rOut = ray.ox < BV_min_x[tcr] | ray.ox > BV_max_x[tcr] |
                        ray.oy < BV_min_y[tcr] | ray.oy > BV_max_y[tcr] |
                        ray.oz < BV_min_z[tcr] | ray.oz > BV_max_z[tcr];
            bool rMiss = tmaxr < tminr | tminr < 0;
            float rDist = (-rMiss + tminr * !rMiss) * rOut;


            // Child ordering for closer intersection and early exit
            bool lcloser = lDist < rDist;

            nstack[ns_top] = tcr * lcloser + tcl * !lcloser;
            ns_top += (rDist >= 0) * lcloser + (lDist >= 0) * !lcloser;

            nstack[ns_top] = tcl * lcloser + tcr * !lcloser;
            ns_top += (lDist >= 0) * lcloser + (rDist >= 0) * !lcloser;

            continue;
        }

        for (int i = BV_pl[nidx]; i < BV_pr[nidx]; ++i) {
            int gi = BV_fi[i];

            bool hit = true;

            int fv0 = MS_fv0[gi],
                fv1 = MS_fv1[gi],
                fv2 = MS_fv2[gi];

            float e1x = MS_vx[fv1] - MS_vx[fv0];
            float e1y = MS_vy[fv1] - MS_vy[fv0];
            float e1z = MS_vz[fv1] - MS_vz[fv0];

            float e2x = MS_vx[fv2] - MS_vx[fv0];
            float e2y = MS_vy[fv2] - MS_vy[fv0];
            float e2z = MS_vz[fv2] - MS_vz[fv0];

            float hx = ray.dy * e2z - ray.dz * e2y;
            float hy = ray.dz * e2x - ray.dx * e2z;
            float hz = ray.dx * e2y - ray.dy * e2x;

            float a = e1x * hx + e1y * hy + e1z * hz;

            hit &= a != 0.0f;
            a += !hit; // Avoid 0 division

            float sx = ray.ox - MS_vx[fv0];
            float sy = ray.oy - MS_vy[fv0];
            float sz = ray.oz - MS_vz[fv0];

            float f = 1.0f / a;

            float u = f * (sx * hx + sy * hy + sz * hz);

            hit &= u >= 0.0f & u <= 1.0f;

            float qx = sy * e1z - sz * e1y;
            float qy = sz * e1x - sx * e1z;
            float qz = sx * e1y - sy * e1x;

            float v = f * (ray.dx * qx + ray.dy * qy + ray.dz * qz);
            float w = 1.0f - u - v;

            hit &= v >= 0.0f & w >= 0.0f;

            float t = f * (e2x * qx + e2y * qy + e2z * qz);

            hit &= t > 0.0f & t < H_t;

            // Remove this: a little trick used to get wireframe
            // hit &= v < 0.01f | w < 0.01f | u < 0.01f;

            H_t = t * hit + H_t * !hit;
            H_u = u * hit + H_u * !hit;
            H_v = v * hit + H_v * !hit;
            H_w = w * hit + H_w * !hit;
            H_Idx = gi * hit + H_Idx * !hit;
        }
    }

    if (H_Idx == -1) {
        // Mess around with these values for fun
        // float3 ground = { 0.01f, 0.01f, 0.03f };
        // float3 skyHorizon = { 0.01f, 0.01f, 0.03f };
        // float3 skyZenith = { 0.00f, 0.00f, 0.00f };
        // float3 sunDir = { -1, -1, 1 };
        // float sunFocus = 169.0f, sunIntensity = 0.6f;

        float3 ground = { 1.00f, 1.00f, 1.00f };
        float3 skyHorizon = { 0.70f, 0.70f, 0.70f };
        float3 skyZenith = { 0.10f, 0.20f, 0.90f };
        float3 sunDir = { -1, -1, 1 };
        float sunFocus = 100.0f, sunIntensity = 8.0f;

        // float sunMag = sqrtf(sunDir.x * sunDir.x + sunDir.y * sunDir.y + sunDir.z * sunDir.z);
        float rsunMag = AzDevMath::rsqrt(sunDir.x * sunDir.x + sunDir.y * sunDir.y + sunDir.z * sunDir.z);
        sunDir.x *= rsunMag; sunDir.y *= rsunMag; sunDir.z *= rsunMag;

        // Sky calculation
        float sky_t = ray.dy * 2.2f;
        sky_t = fmaxf(0.0f, fminf(1.0f, sky_t));
        float skyGradT = powf(sky_t, 0.35f);
        float skyGradR = skyHorizon.x * (1.0f - skyGradT) + skyZenith.x * skyGradT;
        float skyGradG = skyHorizon.y * (1.0f - skyGradT) + skyZenith.y * skyGradT;
        float skyGradB = skyHorizon.z * (1.0f - skyGradT) + skyZenith.z * skyGradT;

        // Sun calculation
        float SdotR = sunDir.x * ray.dx + sunDir.y * ray.dy + sunDir.z * ray.dz;
        SdotR *= -(SdotR < 0.0f);
        float sun_t = powf(SdotR, sunFocus) * sunIntensity;
        bool sky_mask = ray.dy > 0.0f;

        // Final color calculation
        float final_r = ground.x * !sky_mask + (skyGradR + sun_t) * sky_mask;
        float final_g = ground.y * !sky_mask + (skyGradG + sun_t) * sky_mask;
        float final_b = ground.z * !sky_mask + (skyGradB + sun_t) * sky_mask;

        final_r = fmaxf(0.0f, fminf(1.0f, final_r));
        final_g = fmaxf(0.0f, fminf(1.0f, final_g));
        final_b = fmaxf(0.0f, fminf(1.0f, final_b));

        frmx[tIdx] = fakeShading * final_r;
        frmy[tIdx] = fakeShading * final_g;
        frmz[tIdx] = fakeShading * final_b;
        frmdepth[tIdx] = -1.0f;
        frmmat[tIdx] = -1;
        return;
    }

    const AzMtl &H_m = mats[MS_fm[H_Idx]];

    // Texture interpolation (if available)
    int H_ft0 = MS_ft0[H_Idx], H_ft1 = MS_ft1[H_Idx], H_ft2 = MS_ft2[H_Idx];
    float H_tu = MS_tx[H_ft0] * H_w + MS_tx[H_ft1] * H_u + MS_tx[H_ft2] * H_v;
    float H_tv = MS_ty[H_ft0] * H_w + MS_ty[H_ft1] * H_u + MS_ty[H_ft2] * H_v;
    H_tu -= floor(H_tu); H_tv -= floor(H_tv);

    int H_alb_map = H_m.AlbMap,
        H_tw = TX_w[H_alb_map],
        H_th = TX_h[H_alb_map],
        H_toff = TX_off[H_alb_map];

    int H_tx = (int)(H_tu * H_tw),
        H_ty = (int)(H_tv * H_th),
        H_tIdx = H_toff + H_ty * H_tw + H_tx;

    bool H_hasT = H_m.AlbMap > 0;
    float H_alb_r = TX_r[H_tIdx] * H_hasT + H_m.Alb_r * !H_hasT;
    float H_alb_g = TX_g[H_tIdx] * H_hasT + H_m.Alb_g * !H_hasT;
    float H_alb_b = TX_b[H_tIdx] * H_hasT + H_m.Alb_b * !H_hasT;

    // Normal interpolation
    int H_fn0 = MS_fn0[H_Idx], H_fn1 = MS_fn1[H_Idx], H_fn2 = MS_fn2[H_Idx];
    float H_nx = MS_nx[H_fn0] * H_w + MS_nx[H_fn1] * H_u + MS_nx[H_fn2] * H_v;
    float H_ny = MS_ny[H_fn0] * H_w + MS_ny[H_fn1] * H_u + MS_ny[H_fn2] * H_v;
    float H_nz = MS_nz[H_fn0] * H_w + MS_nz[H_fn1] * H_u + MS_nz[H_fn2] * H_v;
    bool H_hasN = H_fn0 > 0;

    // Fake shading
    bool fShade = fakeShading & H_hasN;
    float H_NdotR_D = H_nx * ray.dx + H_ny * ray.dy + H_nz * ray.dz;
    H_NdotR_D = (0.4 + H_NdotR_D * H_NdotR_D * 0.6) * fShade + !fShade;

    frmx[tIdx] = H_alb_r * H_NdotR_D;
    frmy[tIdx] = H_alb_g * H_NdotR_D;
    frmz[tIdx] = H_alb_b * H_NdotR_D;
    frmdepth[tIdx] = H_t;
    frmmat[tIdx] = MS_fm[H_Idx];
};