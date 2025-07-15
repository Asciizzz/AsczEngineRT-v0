#include <PathTraceNEE.cuh>
#include <AzDevMath.cuh>

__device__ inline float rnd(uint32_t &seed) {
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;

    return (seed * 2.3283064365386963e-10f); // 1 / (2^32)
}



__global__ void pathtraceNEEKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,

    float *MS_vx, float *MS_vy, float *MS_vz, float *MS_tx, float *MS_ty, float *MS_nx, float *MS_ny, float *MS_nz,
    int *MS_fv0, int *MS_fv1, int *MS_fv2, int *MS_ft0, int *MS_ft1, int *MS_ft2, int *MS_fn0, int *MS_fn1, int *MS_fn2, int *MS_fm,
    int *MS_lsrc, int MS_lnum,

    // AzMtl *mats, int *lsrc, int lNum,
    float *MT_alb_r, float *MT_alb_g, float *MT_alb_b, int *MT_alb_map,
    float *MT_rough, float *MT_metal, float *MT_tr, float *MT_ior,
    float *MT_ems_r, float *MT_ems_g, float *MT_ems_b, float *MT_ems_i,

    float *TX_r, float *TX_g, float *TX_b, float *TX_a,
    int *TX_w, int *TX_h, int *TX_off,

    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *BV_pl, int *BV_pr, bool *BV_lf, int *BV_fi,

    uint32_t seed
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmw * frmh) return;

    const int MAX_BOUNCES = 4;
    const int MAX_NODES = 32;

    float R_rndA = rnd(seed);
    float R_rndB = rnd(seed);

    Ray R_cast = camera.castRay(
        tIdx % frmw, tIdx / frmw, frmw, frmh,
        R_rndA, R_rndB
    );

    float R_ox  = R_cast.ox,  R_oy  = R_cast.oy,  R_oz  = R_cast.oz;  // Origin
    float R_dx  = R_cast.dx,  R_dy  = R_cast.dy,  R_dz  = R_cast.dz;  // Direction
    float R_rdx = R_cast.rdx, R_rdy = R_cast.rdy, R_rdz = R_cast.rdz; // Inverse direction
    int RIgnore = -1; // Ignore face index
    // float RIor = R_cast.Ior;     // Index of refraction

    int nstack[MAX_NODES];
    int ns_top = 0;

    float THRU_x = 1.0f, THRU_y = 1.0f, THRU_z = 1.0f; // Throughput
    float RADI_x = 0.0f, RADI_y = 0.0f, RADI_z = 0.0f; // Radiance

    int R_bounce = 0;
    while (R_bounce < MAX_BOUNCES) {
        int H_idx = -1;
        float H_t = 1e9f;
        float H_u = 0.0f;
        float H_v = 0.0f;
        float H_w = 0.0f;

        ns_top = 1;
        nstack[0] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (BV_min_x[nidx] - R_ox) * R_rdx;
            float t2n = (BV_max_x[nidx] - R_ox) * R_rdx;
            float t3n = (BV_min_y[nidx] - R_oy) * R_rdy;
            float t4n = (BV_max_y[nidx] - R_oy) * R_rdy;
            float t5n = (BV_min_z[nidx] - R_oz) * R_rdz;
            float t6n = (BV_max_z[nidx] - R_oz) * R_rdz;

            float tminn1 = fminf(t1n, t2n), tmaxn1 = fmaxf(t1n, t2n);
            float tminn2 = fminf(t3n, t4n), tmaxn2 = fmaxf(t3n, t4n);
            float tminn3 = fminf(t5n, t6n), tmaxn3 = fmaxf(t5n, t6n);
    
            float tminn = fmaxf(fmaxf(tminn1, tminn2), tminn3);
            float tmaxn = fminf(fminf(tmaxn1, tmaxn2), tmaxn3);

            bool nOut = R_ox < BV_min_x[nidx] | R_ox > BV_max_x[nidx] |
                        R_oy < BV_min_y[nidx] | R_oy > BV_max_y[nidx] |
                        R_oz < BV_min_z[nidx] | R_oz > BV_max_z[nidx];
            bool nMiss = tmaxn < tminn | (tminn < 0 & nOut) | tminn > H_t;

            if (nMiss) continue;

            // If node is not a leaf:
            if (!BV_lf[nidx]) {
                // Find the distance to the left child
                int tcl = BV_pl[nidx];
                float t1l = (BV_min_x[tcl] - R_ox) * R_rdx;
                float t2l = (BV_max_x[tcl] - R_ox) * R_rdx;
                float t3l = (BV_min_y[tcl] - R_oy) * R_rdy;
                float t4l = (BV_max_y[tcl] - R_oy) * R_rdy;
                float t5l = (BV_min_z[tcl] - R_oz) * R_rdz;
                float t6l = (BV_max_z[tcl] - R_oz) * R_rdz;

                float tminl1 = fminf(t1l, t2l), tmaxl1 = fmaxf(t1l, t2l);
                float tminl2 = fminf(t3l, t4l), tmaxl2 = fmaxf(t3l, t4l);
                float tminl3 = fminf(t5l, t6l), tmaxl3 = fmaxf(t5l, t6l);
    
                float tminl = fmaxf(fmaxf(tminl1, tminl2), tminl3);
                float tmaxl = fminf(fminf(tmaxl1, tmaxl2), tmaxl3);

                bool lOut = R_ox < BV_min_x[tcl] | R_ox > BV_max_x[tcl] |
                            R_oy < BV_min_y[tcl] | R_oy > BV_max_y[tcl] |
                            R_oz < BV_min_z[tcl] | R_oz > BV_max_z[tcl];
                bool lMiss = tmaxl < tminl | tminl < 0;
                float lDist = (-lMiss + tminl * !lMiss) * lOut;

                // Find the distance to the right child
                int tcr = BV_pr[nidx];
                float t1r = (BV_min_x[tcr] - R_ox) * R_rdx;
                float t2r = (BV_max_x[tcr] - R_ox) * R_rdx;
                float t3r = (BV_min_y[tcr] - R_oy) * R_rdy;
                float t4r = (BV_max_y[tcr] - R_oy) * R_rdy;
                float t5r = (BV_min_z[tcr] - R_oz) * R_rdz;
                float t6r = (BV_max_z[tcr] - R_oz) * R_rdz;

                float tminr1 = fminf(t1r, t2r), tmaxr1 = fmaxf(t1r, t2r);
                float tminr2 = fminf(t3r, t4r), tmaxr2 = fmaxf(t3r, t4r);
                float tminr3 = fminf(t5r, t6r), tmaxr3 = fmaxf(t5r, t6r);
    
                float tminr = fmaxf(fmaxf(tminr1, tminr2), tminr3);
                float tmaxr = fminf(fminf(tmaxr1, tmaxr2), tmaxr3);

                bool rOut = R_ox < BV_min_x[tcr] | R_ox > BV_max_x[tcr] |
                            R_oy < BV_min_y[tcr] | R_oy > BV_max_y[tcr] |
                            R_oz < BV_min_z[tcr] | R_oz > BV_max_z[tcr];
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
                int fi = BV_fi[i];

                bool hit = fi != RIgnore;

                int fv0 = MS_fv0[fi],
                    fv1 = MS_fv1[fi],
                    fv2 = MS_fv2[fi];

                float e1x = MS_vx[fv1] - MS_vx[fv0];
                float e1y = MS_vy[fv1] - MS_vy[fv0];
                float e1z = MS_vz[fv1] - MS_vz[fv0];

                float e2x = MS_vx[fv2] - MS_vx[fv0];
                float e2y = MS_vy[fv2] - MS_vy[fv0];
                float e2z = MS_vz[fv2] - MS_vz[fv0];

                float hx = R_dy * e2z - R_dz * e2y;
                float hy = R_dz * e2x - R_dx * e2z;
                float hz = R_dx * e2y - R_dy * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a += !hit;

                float sx = R_ox - MS_vx[fv0];
                float sy = R_oy - MS_vy[fv0];
                float sz = R_oz - MS_vz[fv0];

                float f = 1.0f / a;

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (R_dx * qx + R_dy * qy + R_dz * qz);
                float w = 1.0f - u - v;

                hit &= v >= 0.0f & w >= 0.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < H_t;

                H_t = t * hit + H_t * !hit;
                H_u = u * hit + H_u * !hit;
                H_v = v * hit + H_v * !hit;
                H_w = w * hit + H_w * !hit;
                H_idx = fi * hit + H_idx * !hit;
            }
        }

        if (H_idx == -1) {
            // break;

            float3 sunDir = { -1, -1, 1 };
            float3 ground = { 0.00f, 0.00f, 0.00f };

            float3 skyHorizon = { 1.00f, 1.00f, 1.00f };
            float3 skyZenith = { 0.10f, 0.20f, 0.90f };
            // float3 skyZenith = { 0.20f, 1.00f, 0.30f };
            float sunFocus = 100.0f, sunIntensity = 10.0f;

            // float3 skyHorizon = { 0.00f, 0.00f, 0.00f };
            // float3 skyZenith = { 0.00f, 0.00f, 0.00f };
            // float sunFocus = 200.0f, sunIntensity = 0.3f;

            // float sunMag = sqrtf(sunDir.x * sunDir.x + sunDir.y * sunDir.y + sunDir.z * sunDir.z);
            float rsunMag = AzDevMath::rsqrt(sunDir.x * sunDir.x + sunDir.y * sunDir.y + sunDir.z * sunDir.z);
            sunDir.x *= rsunMag; sunDir.y *= rsunMag; sunDir.z *= rsunMag;

            // Sky calculation
            float sky_t = R_dy * 2.2f;
            sky_t = fmaxf(0.0f, fminf(1.0f, sky_t));
            float skyGradT = powf(sky_t, 0.35f);
            float skyGradR = skyHorizon.x * (1.0f - skyGradT) + skyZenith.x * skyGradT;
            float skyGradG = skyHorizon.y * (1.0f - skyGradT) + skyZenith.y * skyGradT;
            float skyGradB = skyHorizon.z * (1.0f - skyGradT) + skyZenith.z * skyGradT;

            // Sun calculation
            float SdotR = sunDir.x * R_dx + sunDir.y * R_dy + sunDir.z * R_dz;
            SdotR *= -(SdotR < 0.0f);
            float sun_t = powf(SdotR, sunFocus) * sunIntensity;
            bool sky_mask = R_dy > 0.0f;

            // // Star calculation
            // float theta = atan2f(R_dz, R_dx);
            // float phi = acosf(R_dy);

            float final_r = ground.x * !sky_mask + (skyGradR + sun_t) * sky_mask;
            float final_g = ground.y * !sky_mask + (skyGradG + sun_t) * sky_mask;
            float final_b = ground.z * !sky_mask + (skyGradB + sun_t) * sky_mask;

            RADI_x += final_r * THRU_x;
            RADI_y += final_g * THRU_y;
            RADI_z += final_b * THRU_z;

            break;
        }

        int H_fm = MS_fm[H_idx];

        // Vertex linear interpolation
        float H_vx = R_ox + R_dx * H_t;
        float H_vy = R_oy + R_dy * H_t;
        float H_vz = R_oz + R_dz * H_t;

        // Texture interpolation (if available)
        int H_ft0 = MS_ft0[H_idx], H_ft1 = MS_ft1[H_idx], H_ft2 = MS_ft2[H_idx];
        float H_tu = MS_tx[H_ft0] * H_w + MS_tx[H_ft1] * H_u + MS_tx[H_ft2] * H_v;
        float H_tv = MS_ty[H_ft0] * H_w + MS_ty[H_ft1] * H_u + MS_ty[H_ft2] * H_v;
        H_tu -= floor(H_tu); H_tv -= floor(H_tv);

        int H_alb_map = MT_alb_map[H_fm],
            H_tw = TX_w[H_alb_map],
            H_th = TX_h[H_alb_map],
            H_toff = TX_off[H_alb_map];

        int H_tx = (int)(H_tu * H_tw),
            H_ty = (int)(H_tv * H_th),
            H_tIdx = H_toff + H_ty * H_tw + H_tx;

        bool H_hasT = H_alb_map > 0;
        float H_alb_r = TX_r[H_tIdx] * H_hasT + MT_alb_r[H_fm] * !H_hasT;
        float H_alb_g = TX_g[H_tIdx] * H_hasT + MT_alb_g[H_fm] * !H_hasT;
        float H_alb_b = TX_b[H_tIdx] * H_hasT + MT_alb_b[H_fm] * !H_hasT;

        // Normal interpolation
        int H_fn0 = MS_fn0[H_idx], H_fn1 = MS_fn1[H_idx], H_fn2 = MS_fn2[H_idx];
        float H_nx = MS_nx[H_fn0] * H_w + MS_nx[H_fn1] * H_u + MS_nx[H_fn2] * H_v;
        float H_ny = MS_ny[H_fn0] * H_w + MS_ny[H_fn1] * H_u + MS_ny[H_fn2] * H_v;
        float H_nz = MS_nz[H_fn0] * H_w + MS_nz[H_fn1] * H_u + MS_nz[H_fn2] * H_v;
        bool H_hasN = H_fn0 > 0;

// =================== Direct lighting =========================

    // Sample random light source
        int DL_idx = MS_lnum ? MS_lsrc[(int)(MS_lnum * rnd(seed))] : 0;
        int DL_fm = MS_fm[DL_idx];

        // Sample random point on the light source
        float DL_u = rnd(seed);
        float DL_v = rnd(seed);
        bool DL_uv_valid = DL_u + DL_v < 1.0f;

        DL_u = DL_u * DL_uv_valid + (1.0f - DL_u) * !DL_uv_valid;
        DL_v = DL_v * DL_uv_valid + (1.0f - DL_v) * !DL_uv_valid;
        float DL_w = 1.0f - DL_u - DL_v;

    // Sample light's vertex
        int DL_fv0 = MS_fv0[DL_idx], DL_fv1 = MS_fv1[DL_idx], DL_fv2 = MS_fv2[DL_idx];
        float DL_vx = MS_vx[DL_fv0] * DL_w + MS_vx[DL_fv1] * DL_u + MS_vx[DL_fv2] * DL_v;
        float DL_vy = MS_vy[DL_fv0] * DL_w + MS_vy[DL_fv1] * DL_u + MS_vy[DL_fv2] * DL_v;
        float DL_vz = MS_vz[DL_fv0] * DL_w + MS_vz[DL_fv1] * DL_u + MS_vz[DL_fv2] * DL_v;

    // Sample light's albedo
        int DL_ft0 = MS_ft0[DL_idx], DL_ft1 = MS_ft1[DL_idx], DL_ft2 = MS_ft2[DL_idx];
        float DL_tu = MS_tx[DL_ft0] * DL_w + MS_tx[DL_ft1] * DL_u + MS_tx[DL_ft2] * DL_v;
        float DL_tv = MS_ty[DL_ft0] * DL_w + MS_ty[DL_ft1] * DL_u + MS_ty[DL_ft2] * DL_v;

        int DL_alb_map = MT_alb_map[DL_fm],
            DL_tw = TX_w[DL_alb_map],
            DL_th = TX_h[DL_alb_map],
            DL_toff = TX_off[DL_alb_map];

        int DL_tx = (int)(DL_tu * DL_tw),
            DL_ty = (int)(DL_tv * DL_th),
            DL_tIdx = DL_toff + DL_ty * DL_tw + DL_tx;

        bool DL_hasT = DL_alb_map > 0;
        float DL_alb_r = TX_r[DL_tIdx] * DL_hasT + MT_alb_r[DL_fm] * !DL_hasT;
        float DL_alb_g = TX_g[DL_tIdx] * DL_hasT + MT_alb_g[DL_fm] * !DL_hasT;
        float DL_alb_b = TX_b[DL_tIdx] * DL_hasT + MT_alb_b[DL_fm] * !DL_hasT;

    // Sample light's direction (not normalized)
        float DL_dx = H_vx - DL_vx;
        float DL_dy = H_vy - DL_vy;
        float DL_dz = H_vz - DL_vz;

    // Sample light's distance
        float DL_distSqr = DL_dx * DL_dx + DL_dy * DL_dy + DL_dz * DL_dz;
        float DL_distRsqrt = AzDevMath::rsqrt(DL_distSqr + !DL_distSqr); // Avoid zero division
        float DL_dist = DL_distSqr * DL_distRsqrt;

        // Normalize light direction
        DL_dx *= DL_distRsqrt;
        DL_dy *= DL_distRsqrt;
        DL_dz *= DL_distRsqrt;

        // Sample light's inverse direction (for traversal)
        float DL_rdx = 1.0f / DL_dx;
        float DL_rdy = 1.0f / DL_dy;
        float DL_rdz = 1.0f / DL_dz;

    // Get relevant data
        float DL_NdotH_N = DL_dx * H_nx + DL_dy * H_ny + DL_dz * H_nz;
        DL_NdotH_N = -DL_NdotH_N * (DL_NdotH_N < 0.0f) + !H_hasN;

    // Check for occlusion
        ns_top = 1;
        nstack[0] = 0;

        bool occluded = !DL_idx;
        while (ns_top > 0 && !occluded) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (BV_min_x[nidx] - DL_vx) * DL_rdx;
            float t2n = (BV_max_x[nidx] - DL_vx) * DL_rdx;
            float t3n = (BV_min_y[nidx] - DL_vy) * DL_rdy;
            float t4n = (BV_max_y[nidx] - DL_vy) * DL_rdy;
            float t5n = (BV_min_z[nidx] - DL_vz) * DL_rdz;
            float t6n = (BV_max_z[nidx] - DL_vz) * DL_rdz;

            float tminn1 = fminf(t1n, t2n), tmaxn1 = fmaxf(t1n, t2n);
            float tminn2 = fminf(t3n, t4n), tmaxn2 = fmaxf(t3n, t4n);
            float tminn3 = fminf(t5n, t6n), tmaxn3 = fmaxf(t5n, t6n);

            float tminn = fmaxf(fmaxf(tminn1, tminn2), tminn3);
            float tmaxn = fminf(fminf(tmaxn1, tmaxn2), tmaxn3);

            bool nOut = DL_vx < BV_min_x[nidx] | DL_vx > BV_max_x[nidx] |
                        DL_vy < BV_min_y[nidx] | DL_vy > BV_max_y[nidx] |
                        DL_vz < BV_min_z[nidx] | DL_vz > BV_max_z[nidx];
            bool nMiss = tmaxn < tminn | (tminn < 0 & nOut) | tminn > DL_dist;

            if (nMiss) continue;

            // If node is not a leaf:
            if (!BV_lf[nidx]) {
                // Find the distance to the left child
                int tcl = BV_pl[nidx];
                float t1l = (BV_min_x[tcl] - DL_vx) * DL_rdx;
                float t2l = (BV_max_x[tcl] - DL_vx) * DL_rdx;
                float t3l = (BV_min_y[tcl] - DL_vy) * DL_rdy;
                float t4l = (BV_max_y[tcl] - DL_vy) * DL_rdy;
                float t5l = (BV_min_z[tcl] - DL_vz) * DL_rdz;
                float t6l = (BV_max_z[tcl] - DL_vz) * DL_rdz;

                float tminl1 = fminf(t1l, t2l), tmaxl1 = fmaxf(t1l, t2l);
                float tminl2 = fminf(t3l, t4l), tmaxl2 = fmaxf(t3l, t4l);
                float tminl3 = fminf(t5l, t6l), tmaxl3 = fmaxf(t5l, t6l);

                float tminl = fmaxf(fmaxf(tminl1, tminl2), tminl3);
                float tmaxl = fminf(fminf(tmaxl1, tmaxl2), tmaxl3);

                bool lOut = DL_vx < BV_min_x[tcl] | DL_vx > BV_max_x[tcl] |
                            DL_vy < BV_min_y[tcl] | DL_vy > BV_max_y[tcl] |
                            DL_vz < BV_min_z[tcl] | DL_vz > BV_max_z[tcl];
                bool lMiss = tmaxl < tminl | tminl < 0;
                float lDist = (-lMiss + tminl * !lMiss) * lOut;

                // Find the distance to the right child
                int tcr = BV_pr[nidx];
                float t1r = (BV_min_x[tcr] - DL_vx) * DL_rdx;
                float t2r = (BV_max_x[tcr] - DL_vx) * DL_rdx;
                float t3r = (BV_min_y[tcr] - DL_vy) * DL_rdy;
                float t4r = (BV_max_y[tcr] - DL_vy) * DL_rdy;
                float t5r = (BV_min_z[tcr] - DL_vz) * DL_rdz;
                float t6r = (BV_max_z[tcr] - DL_vz) * DL_rdz;

                float tminr1 = fminf(t1r, t2r), tmaxr1 = fmaxf(t1r, t2r);
                float tminr2 = fminf(t3r, t4r), tmaxr2 = fmaxf(t3r, t4r);
                float tminr3 = fminf(t5r, t6r), tmaxr3 = fmaxf(t5r, t6r);

                float tminr = fmaxf(fmaxf(tminr1, tminr2), tminr3);
                float tmaxr = fminf(fminf(tmaxr1, tmaxr2), tmaxr3);

                bool rOut = DL_vx < BV_min_x[tcr] | DL_vx > BV_max_x[tcr] |
                            DL_vy < BV_min_y[tcr] | DL_vy > BV_max_y[tcr] |
                            DL_vz < BV_min_z[tcr] | DL_vz > BV_max_z[tcr];
                bool rMiss = tmaxr < tminr | tminr < 0;
                float rDist = (-rMiss + tminr * !rMiss) * rOut;

                // No child ordering required for anyHit()
                nstack[ns_top] = tcl;
                ns_top += lDist >= 0;
                nstack[ns_top] = tcr;
                ns_top += rDist >= 0;

                continue;
            }

            for (int i = BV_pl[nidx]; i < BV_pr[nidx] & !occluded; ++i) {
                int fi = BV_fi[i];

                bool hit = fi != RIgnore & fi != H_idx;

                int fv0 = MS_fv0[fi],
                    fv1 = MS_fv1[fi],
                    fv2 = MS_fv2[fi];

                float e1x = MS_vx[fv1] - MS_vx[fv0];
                float e1y = MS_vy[fv1] - MS_vy[fv0];
                float e1z = MS_vz[fv1] - MS_vz[fv0];

                float e2x = MS_vx[fv2] - MS_vx[fv0];
                float e2y = MS_vy[fv2] - MS_vy[fv0];
                float e2z = MS_vz[fv2] - MS_vz[fv0];

                float hx = DL_dy * e2z - DL_dz * e2y;
                float hy = DL_dz * e2x - DL_dx * e2z;
                float hz = DL_dx * e2y - DL_dy * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a;

                float sx = DL_vx - MS_vx[fv0];
                float sy = DL_vy - MS_vy[fv0];
                float sz = DL_vz - MS_vz[fv0];

                float f = 1.0f / a;

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (DL_dx * qx + DL_dy * qy + DL_dz * qz);
                float w = 1.0f - u - v;

                hit &= v >= 0.0f & w >= 0.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < DL_dist;

                occluded |= hit;
                ns_top *= !hit;
            }
        }

    // Add direct lighting
        float RADI_i = DL_NdotH_N * MT_ems_i[DL_fm] * !occluded;
        RADI_x += THRU_x * MT_ems_r[DL_fm] * DL_alb_r * H_alb_r * RADI_i + THRU_x * MT_ems_r[H_fm] * MT_ems_i[H_fm] * H_alb_r;
        RADI_y += THRU_y * MT_ems_g[DL_fm] * DL_alb_g * H_alb_g * RADI_i + THRU_y * MT_ems_g[H_fm] * MT_ems_i[H_fm] * H_alb_g;
        RADI_z += THRU_z * MT_ems_b[DL_fm] * DL_alb_b * H_alb_b * RADI_i + THRU_z * MT_ems_b[H_fm] * MT_ems_i[H_fm] * H_alb_b;

        float H_Tr = MT_tr[H_fm];
        THRU_x *= H_alb_r * (1.0f - H_Tr) + H_Tr;
        THRU_y *= H_alb_g * (1.0f - H_Tr) + H_Tr;
        THRU_z *= H_alb_b * (1.0f - H_Tr) + H_Tr;


// =================== Indirect lighting =========================

        // Random diffuse lighting
        float IL_rndA = rnd(seed);
        float IL_rndB = rnd(seed);

        float IL_theta1 = acosf(sqrtf(1.0f - IL_rndA));
        float IL_sinTheta1 = sinf(IL_theta1);

        float IL_phi = M_PIx2 * IL_rndB;
        float IL_sinPhi = sinf(IL_phi);
        float IL_cosPhi = cosf(IL_phi);

        // Cosine weighted hemisphere
        float IL_rnd_x = IL_sinTheta1 * IL_cosPhi;
        float IL_rnd_y = IL_sinTheta1 * IL_sinPhi;
        float IL_rnd_z = cosf(IL_theta1);

        // Truly random direction
        float IL_theta2 = acosf(1.0f - 2.0f * IL_rndA);
        float IL_sinTheta2 = sinf(IL_theta2);

        float IL_truly_rnd_x = IL_sinTheta2 * IL_cosPhi;
        float IL_truly_rnd_y = IL_sinTheta2 * IL_sinPhi;
        float IL_truly_rnd_z = cosf(IL_theta2);

        // Construct a coordinate system
        bool IL_xGreater = fabsf(H_nx) > 0.9;

        // Tangent vector
        // There supposed to also be a ta_z, but since its = 0,
        // you can ignore it in the cross product calculation
        float IL_tang_x = IL_xGreater * H_nz;
        float IL_tang_y = (IL_xGreater - 1) * H_nz;
        float IL_tang_z = (1 - IL_xGreater) * H_ny - IL_xGreater * H_nx;

        // Bitangent vector
        float IL_bitang_x = IL_tang_y * H_nz - IL_tang_z * H_ny;
        float IL_bitang_y = IL_tang_z * H_nx - IL_tang_x * H_nz;
        float IL_bitang_z = IL_tang_x * H_ny - IL_tang_y * H_nx;

        // Transform the vector to the normal space
        float IL_diff_x = IL_rnd_x * IL_tang_x + IL_rnd_y * IL_bitang_x + IL_rnd_z * H_nx;
        float IL_diff_y = IL_rnd_x * IL_tang_y + IL_rnd_y * IL_bitang_y + IL_rnd_z * H_ny;
        float IL_diff_z = IL_rnd_x * IL_tang_z + IL_rnd_y * IL_bitang_z + IL_rnd_z * H_nz;

    // Specular direction (a.k.a. reflection)
        float IL_spec_x = R_dx - H_nx * 2.0f * (H_nx * R_dx);
        float IL_spec_y = R_dy - H_ny * 2.0f * (H_ny * R_dy);
        float IL_spec_z = R_dz - H_nz * 2.0f * (H_nz * R_dz);

    // Lerp diffuse and specular from roughness
        float IL_rough = MT_rough[H_fm];
        float IL_smooth = 1.0f - IL_rough;
        float IL_r_dx = IL_diff_x * IL_rough + IL_spec_x * IL_smooth;
        float IL_r_dy = IL_diff_y * IL_rough + IL_spec_y * IL_smooth;
        float IL_r_dz = IL_diff_z * IL_rough + IL_spec_z * IL_smooth;

        bool IL_hasTr = IL_rndA < H_Tr;
        IL_r_dx = IL_r_dx * !IL_hasTr + R_dx * IL_hasTr;
        IL_r_dy = IL_r_dy * !IL_hasTr + R_dy * IL_hasTr;
        IL_r_dz = IL_r_dz * !IL_hasTr + R_dz * IL_hasTr;

// =================== Construct new ray =========================

        // Origin (truly random for non-normal surfaces)
        R_ox = H_vx;
        R_oy = H_vy;
        R_oz = H_vz;
        // Direction
        R_dx = IL_r_dx * H_hasN + IL_truly_rnd_x * !H_hasN;
        R_dy = IL_r_dy * H_hasN + IL_truly_rnd_y * !H_hasN;
        R_dz = IL_r_dz * H_hasN + IL_truly_rnd_z * !H_hasN;
        // Inverse direction
        R_rdx = 1.0f / R_dx;
        R_rdy = 1.0f / R_dy;
        R_rdz = 1.0f / R_dz;
        // Other ray properties
        RIgnore = H_idx;
        // RIor = H_m.Ior;

// =================== RUSSIAN ROULETTE TERMINATION =========================

        R_bounce += (H_Tr == 0.0f);

        // float R_survival = fminf(1.0f, // Luminance
        //     0.2126f * THRU_x + 0.7152f * THRU_y + 0.0722f * THRU_z
        // );

        // float R_rsurvival = 1.0f / R_survival;

        // bool R_survived = curand_uniform(&rnd[tIdx]) < R_survival;

        // R_bounce += 1 + !R_survived * MAX_BOUNCES - (H_Tr > 0.0f);

        // // Boost for the surviving ray
        // THRU_x *= R_rsurvival;
        // THRU_y *= R_rsurvival;
        // THRU_z *= R_rsurvival;
    }

    frmx[tIdx] = RADI_x;
    frmy[tIdx] = RADI_y;
    frmz[tIdx] = RADI_z;
}
