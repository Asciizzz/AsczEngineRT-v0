#include <PathTraceNEE.cuh>
#include <AzDevMath.cuh>

__global__ void pathtraceNEEKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats, int *lsrc, int lNum,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    curandState *rnd
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmw * frmh) return;

    const int MAX_BOUNCES = 4;
    const int MAX_NODES = 64;

    int tX = tIdx % frmw;
    int tY = tIdx / frmw;

    float R_rndA = curand_uniform(&rnd[tIdx]);
    float R_rndB = curand_uniform(&rnd[tIdx]);

    Ray ray = camera.castRay(tX, tY, frmw, frmh, R_rndA, R_rndB);

    float R_ox  = ray.ox,  R_oy  = ray.oy,  R_oz  = ray.oz;  // Origin
    float R_dx  = ray.dx,  R_dy  = ray.dy,  R_dz  = ray.dz;  // Direction
    float R_rdx = ray.rdx, R_rdy = ray.rdy, R_rdz = ray.rdz; // Inverse direction
    int RIgnore = ray.ignore; // Ignore face index
    // float RIor = ray.Ior;     // Index of refraction

    int nstack[MAX_NODES];
    int ns_top = 0;

    float THRU_x = 1.0f, THRU_y = 1.0f, THRU_z = 1.0f; // Throughput
    float RADI_x = 0.0f, RADI_y = 0.0f, RADI_z = 0.0f; // Radiance

    int R_bounce = 0;
    while (R_bounce < MAX_BOUNCES) {
        int H_Idx = -1;
        float H_t = 1e9f;
        float H_u = 0.0f;
        float H_v = 0.0f;
        float H_w = 0.0f;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (mi_x[nidx] - R_ox) * R_rdx;
            float t2n = (mx_x[nidx] - R_ox) * R_rdx;
            float t3n = (mi_y[nidx] - R_oy) * R_rdy;
            float t4n = (mx_y[nidx] - R_oy) * R_rdy;
            float t5n = (mi_z[nidx] - R_oz) * R_rdz;
            float t6n = (mx_z[nidx] - R_oz) * R_rdz;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = R_ox < mi_x[nidx] | R_ox > mx_x[nidx] |
                        R_oy < mi_y[nidx] | R_oy > mx_y[nidx] |
                        R_oz < mi_z[nidx] | R_oz > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > H_t) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - R_ox) * R_rdx;
                float t2l = (mx_x[tcl] - R_ox) * R_rdx;
                float t3l = (mi_y[tcl] - R_oy) * R_rdy;
                float t4l = (mx_y[tcl] - R_oy) * R_rdy;
                float t5l = (mi_z[tcl] - R_oz) * R_rdz;
                float t6l = (mx_z[tcl] - R_oz) * R_rdz;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = R_ox < mi_x[tcl] | R_ox > mx_x[tcl] |
                            R_oy < mi_y[tcl] | R_oy > mx_y[tcl] |
                            R_oz < mi_z[tcl] | R_oz > mx_z[tcl];
                float Ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - R_ox) * R_rdx;
                float t2r = (mx_x[tcr] - R_ox) * R_rdx;
                float t3r = (mi_y[tcr] - R_oy) * R_rdy;
                float t4r = (mx_y[tcr] - R_oy) * R_rdy;
                float t5r = (mi_z[tcr] - R_oz) * R_rdz;
                float t6r = (mx_z[tcr] - R_oz) * R_rdz;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = R_ox < mi_x[tcr] | R_ox > mx_x[tcr] |
                            R_oy < mi_y[tcr] | R_oy > mx_y[tcr] |
                            R_oz < mi_z[tcr] | R_oz > mx_z[tcr];
                float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


                // Child ordering for closer intersection and early exit
                bool lcloser = Ldist < rdist;

                nstack[ns_top] = tcr * lcloser + tcl * !lcloser;
                ns_top += (rdist >= 0) * lcloser + (Ldist >= 0) * !lcloser;

                nstack[ns_top] = tcl * lcloser + tcr * !lcloser;
                ns_top += (Ldist >= 0) * lcloser + (rdist >= 0) * !lcloser;

                continue;
            }

            for (int i = pl[nidx]; i < pr[nidx]; ++i) {
                int gi = gIdx[i];

                bool hit = gi != RIgnore;

                float e1x = vx[fv1[gi]] - vx[fv0[gi]];
                float e1y = vy[fv1[gi]] - vy[fv0[gi]];
                float e1z = vz[fv1[gi]] - vz[fv0[gi]];

                float e2x = vx[fv2[gi]] - vx[fv0[gi]];
                float e2y = vy[fv2[gi]] - vy[fv0[gi]];
                float e2z = vz[fv2[gi]] - vz[fv0[gi]];

                float hx = R_dy * e2z - R_dz * e2y;
                float hy = R_dz * e2x - R_dx * e2z;
                float hz = R_dx * e2y - R_dy * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a;

                float sx = R_ox - vx[fv0[gi]];
                float sy = R_oy - vy[fv0[gi]];
                float sz = R_oz - vz[fv0[gi]];

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
            float3 skyHorizon = { 1.00f, 1.00f, 1.00f };
            float3 skyZenith = { 0.20f, 0.30f, 1.00f };
            float3 sunDir = { -1, -1, 1 };
            float sunFocus = 100.0f, sunIntensity = 8.0f;

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

/* Guide:

H_: hit point
DL_: direct light
IL_: indirect light

*/

        // Get the face material
        const AzMtl &H_m = mats[fm[H_Idx]];

        // Vertex linear interpolation
        float H_vx = R_ox + R_dx * H_t;
        float H_vy = R_oy + R_dy * H_t;
        float H_vz = R_oz + R_dz * H_t;

        // Texture interpolation (if available)
        int ht0 = ft0[H_Idx], ht1 = ft1[H_Idx], ht2 = ft2[H_Idx];
        float H_tu = tx[ht0] * H_w + tx[ht1] * H_u + tx[ht2] * H_v;
        float H_tv = ty[ht0] * H_w + ty[ht1] * H_u + ty[ht2] * H_v;
        H_tu -= floor(H_tu); H_tv -= floor(H_tv);

        int H_alb_map = H_m.AlbMap;
        int H_tw = tw[H_alb_map];
        int H_th = th[H_alb_map];
        int H_toff = toff[H_alb_map];

        int H_tx = (int)(H_tu * H_tw);
        int H_ty = (int)(H_tv * H_th);
        int H_tIdx = H_toff + H_ty * H_tw + H_tx;

        bool H_hasT = H_m.AlbMap > 0;
        float H_alb_x = tr[H_tIdx] * H_hasT + H_m.Alb_r * !H_hasT;
        float H_alb_y = tg[H_tIdx] * H_hasT + H_m.Alb_g * !H_hasT;
        float H_alb_z = tb[H_tIdx] * H_hasT + H_m.Alb_b * !H_hasT;

        // Normal interpolation
        int hn0 = fn0[H_Idx], hn1 = fn1[H_Idx], hn2 = fn2[H_Idx];
        float H_nx = nx[hn0] * H_w + nx[hn1] * H_u + nx[hn2] * H_v;
        float H_ny = ny[hn0] * H_w + ny[hn1] * H_u + ny[hn2] * H_v;
        float H_nz = nz[hn0] * H_w + nz[hn1] * H_u + nz[hn2] * H_v;
        bool H_hasN = hn0 > 0;

// =================== Direct lighting =========================

        // Retrieve the light source and it's informations

        // Sample random light source
        int DL_Idx = lNum ? lsrc[(int)(lNum * curand_uniform(&rnd[tIdx]))] : 0;
        const AzMtl &DL_m = mats[fm[DL_Idx]];

        // Sample random point on the light source
        float DL_u = curand_uniform(&rnd[tIdx]);
        float DL_v = curand_uniform(&rnd[tIdx]);
        bool DL_uv_valid = DL_u + DL_v < 1.0f;

        DL_u = DL_u * DL_uv_valid + (1.0f - DL_u) * !DL_uv_valid;
        DL_v = DL_v * DL_uv_valid + (1.0f - DL_v) * !DL_uv_valid;
        float DL_w = 1.0f - DL_u - DL_v;

        // Sample light's vertex
        int lv0 = fv0[DL_Idx], lv1 = fv1[DL_Idx], lv2 = fv2[DL_Idx];
        float DL_vx = vx[lv0] * DL_u + vx[lv1] * DL_v + vx[lv2] * DL_w;
        float DL_vy = vy[lv0] * DL_u + vy[lv1] * DL_v + vy[lv2] * DL_w;
        float DL_vz = vz[lv0] * DL_u + vz[lv1] * DL_v + vz[lv2] * DL_w;

        // // Sample light's normal
        // int ln0 = fn0[DL_Idx], ln1 = fn1[DL_Idx], ln2 = fn2[DL_Idx];
        // float DL_nx = nx[ln0] * DL_u + nx[ln1] * DL_v + nx[ln2] * DL_w;
        // float DL_ny = ny[ln0] * DL_u + ny[ln1] * DL_v + ny[ln2] * DL_w;
        // float DL_nz = nz[ln0] * DL_u + nz[ln1] * DL_v + nz[ln2] * DL_w;

        // Sample light's direction (not normalized)
        float DL_dx = H_vx - DL_vx;
        float DL_dy = H_vy - DL_vy;
        float DL_dz = H_vz - DL_vz;

        // Sample light's distance
        float LdistSqr = DL_dx * DL_dx + DL_dy * DL_dy + DL_dz * DL_dz;
        float LdistRsqrt = AzDevMath::rsqrt(LdistSqr + !LdistSqr); // Avoid zero division
        float Ldist = LdistSqr * LdistRsqrt;

        // Normalize light direction
        DL_dx *= LdistRsqrt;
        DL_dy *= LdistRsqrt;
        DL_dz *= LdistRsqrt;

        // Sample light's inverse direction (for traversal)
        float DL_rdx = 1.0f / DL_dx;
        float DL_rdy = 1.0f / DL_dy;
        float DL_rdz = 1.0f / DL_dz;

        // Get relevant data
        float DL_NdotH_N = DL_dx * H_nx + DL_dy * H_ny + DL_dz * H_nz;
        DL_NdotH_N = -DL_NdotH_N * (DL_NdotH_N < 0.0f) + !H_hasN;

        // Check for occlusion
        nstack[0] = 0;
        ns_top = DL_Idx > 0;
        bool occluded = false;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (mi_x[nidx] - DL_vx) * DL_rdx;
            float t2n = (mx_x[nidx] - DL_vx) * DL_rdx;
            float t3n = (mi_y[nidx] - DL_vy) * DL_rdy;
            float t4n = (mx_y[nidx] - DL_vy) * DL_rdy;
            float t5n = (mi_z[nidx] - DL_vz) * DL_rdz;
            float t6n = (mx_z[nidx] - DL_vz) * DL_rdz;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = DL_vx < mi_x[nidx] | DL_vx > mx_x[nidx] |
                        DL_vy < mi_y[nidx] | DL_vy > mx_y[nidx] |
                        DL_vz < mi_z[nidx] | DL_vz > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > Ldist) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - DL_vx) * DL_rdx;
                float t2l = (mx_x[tcl] - DL_vx) * DL_rdx;
                float t3l = (mi_y[tcl] - DL_vy) * DL_rdy;
                float t4l = (mx_y[tcl] - DL_vy) * DL_rdy;
                float t5l = (mi_z[tcl] - DL_vz) * DL_rdz;
                float t6l = (mx_z[tcl] - DL_vz) * DL_rdz;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = DL_vx < mi_x[tcl] | DL_vx > mx_x[tcl] |
                            DL_vy < mi_y[tcl] | DL_vy > mx_y[tcl] |
                            DL_vz < mi_z[tcl] | DL_vz > mx_z[tcl];
                float Ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - DL_vx) * DL_rdx;
                float t2r = (mx_x[tcr] - DL_vx) * DL_rdx;
                float t3r = (mi_y[tcr] - DL_vy) * DL_rdy;
                float t4r = (mx_y[tcr] - DL_vy) * DL_rdy;
                float t5r = (mi_z[tcr] - DL_vz) * DL_rdz;
                float t6r = (mx_z[tcr] - DL_vz) * DL_rdz;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = DL_vx < mi_x[tcr] | DL_vx > mx_x[tcr] |
                            DL_vy < mi_y[tcr] | DL_vy > mx_y[tcr] |
                            DL_vz < mi_z[tcr] | DL_vz > mx_z[tcr];
                float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


                // Child ordering for closer intersection and early exit
                bool lcloser = Ldist < rdist;

                nstack[ns_top] = tcr * lcloser + tcl * !lcloser;
                ns_top += (rdist >= 0) * lcloser + (Ldist >= 0) * !lcloser;

                nstack[ns_top] = tcl * lcloser + tcr * !lcloser;
                ns_top += (Ldist >= 0) * lcloser + (rdist >= 0) * !lcloser;

                continue;
            }

            for (int i = pl[nidx]; i < pr[nidx]; ++i) {
                int gi = gIdx[i];

                bool hit = gi != RIgnore & gi != H_Idx;

                int f0 = fv0[gi], f1 = fv1[gi], f2 = fv2[gi];

                float e1x = vx[f1] - vx[f0];
                float e1y = vy[f1] - vy[f0];
                float e1z = vz[f1] - vz[f0];

                float e2x = vx[f2] - vx[f0];
                float e2y = vy[f2] - vy[f0];
                float e2z = vz[f2] - vz[f0];

                float hx = DL_dy * e2z - DL_dz * e2y;
                float hy = DL_dz * e2x - DL_dx * e2z;
                float hz = DL_dx * e2y - DL_dy * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a;

                float sx = DL_vx - vx[f0];
                float sy = DL_vy - vy[f0];
                float sz = DL_vz - vz[f0];

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

                hit &= t > 0.0f & t < Ldist;

                occluded |= hit;
                ns_top *= !hit;
            }
        }

        float RADI_i = DL_NdotH_N * DL_m.Ems_i * !occluded;
        RADI_x += THRU_x * DL_m.Ems_r * H_alb_x * RADI_i + THRU_x * H_m.Ems_r * H_m.Ems_i;
        RADI_y += THRU_y * DL_m.Ems_g * H_alb_y * RADI_i + THRU_x * H_m.Ems_g * H_m.Ems_i;
        RADI_z += THRU_z * DL_m.Ems_b * H_alb_z * RADI_i + THRU_x * H_m.Ems_b * H_m.Ems_i;

        THRU_x *= H_alb_x * (1.0f - H_m.Tr) + H_m.Tr;
        THRU_y *= H_alb_y * (1.0f - H_m.Tr) + H_m.Tr;
        THRU_z *= H_alb_z * (1.0f - H_m.Tr) + H_m.Tr;


// =================== Indirect lighting =========================

        // Random diffuse lighting
        float IL_rndA = curand_uniform(&rnd[tIdx]);
        float IL_rndB = curand_uniform(&rnd[tIdx]);

        float IL_theta1 = acosf(sqrtf(1.0f - IL_rndA));
        float IL_phi = M_PIx2 * IL_rndB;

        // Cosine weighted hemisphere
        float IL_rnd_x = sinf(IL_theta1) * cosf(IL_phi);
        float IL_rnd_y = sinf(IL_theta1) * sinf(IL_phi);
        float IL_rnd_z = cosf(IL_theta1);

        // Truly random direction
        float IL_theta2 = acosf(1.0f - 2.0f * IL_rndA);
        float IL_truly_rnd_x = sinf(IL_theta2) * cosf(IL_phi);
        float IL_truly_rnd_y = sinf(IL_theta2) * sinf(IL_phi);
        float IL_truly_rnd_z = cosf(IL_theta2);

        // Construct a coordinate system
        bool IL_xGreater = fabsf(H_nx) > 0.9;
        float IL_ta_x = !IL_xGreater;
        float IL_ta_y = IL_xGreater;

        // Tangent vector
        // There supposed to also be a ta_z, but since its = 0,
        // you can ignore it in the cross product calculation
        float IL_tang_x =  IL_ta_y * H_nz;
        float IL_tang_y = -IL_ta_x * H_nz;
        float IL_tang_z = IL_ta_x * H_ny - IL_ta_y * H_nx;

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

    // Lerp diffuse and specular from roughness/smoothness
        float IL_smooth = 1.0f - H_m.Rough;
        float IL_r_dx = IL_diff_x * H_m.Rough + IL_spec_x * IL_smooth;
        float IL_r_dy = IL_diff_y * H_m.Rough + IL_spec_y * IL_smooth;
        float IL_r_dz = IL_diff_z * H_m.Rough + IL_spec_z * IL_smooth;

        bool IL_hasTr = IL_rndA < H_m.Tr;
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
        RIgnore = H_Idx;
        // RIor = H_m.Ior;

// =================== RUSSIAN ROULETTE TERMINATION =========================

        float THRU_lumi = 0.2126f * THRU_x + 0.7152f * THRU_y + 0.0722f * THRU_z;

        float R_survival = fmaxf(1.0f, THRU_lumi);
        float R_rsurvival = 1.0f / R_survival;

        bool R_survived = curand_uniform(&rnd[tIdx]) < R_survival;

        R_bounce += 1 + !R_survived * MAX_BOUNCES;

        // Boost for the surviving ray
        THRU_x *= R_rsurvival;
        THRU_y *= R_rsurvival;
        THRU_z *= R_rsurvival;
    }

    frmx[tIdx] = RADI_x;
    frmy[tIdx] = RADI_y;
    frmz[tIdx] = RADI_z;
}
