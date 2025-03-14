#include <RayCast.cuh>

__global__ void raycastKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats, 
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,
    // Fake shading (for better feel since you can get lost in the scene)
    bool fakeShading,
    // Debugging
    float *frmdepth, int *frmmat
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmw * frmh) return;

    int x = tIdx % frmw;
    int y = tIdx / frmw;
    Ray ray = camera.castRay(x, y, frmw, frmh);

    // Hit info
    int H_Idx = -1;
    float H_t = 1e9f;
    float H_u = 0.0f;
    float H_v = 0.0f;
    float H_w = 0.0f;

    const int MAX_NODES = 64;
    int nstack[MAX_NODES] = { 0 };
    int ns_top = 1;

    while (ns_top > 0) {
        int nidx = nstack[--ns_top];

        // Check if the ray is outside the bounding box
        float t1n = (mi_x[nidx] - ray.ox) * ray.rdx;
        float t2n = (mx_x[nidx] - ray.ox) * ray.rdx;
        float t3n = (mi_y[nidx] - ray.oy) * ray.rdy;
        float t4n = (mx_y[nidx] - ray.oy) * ray.rdy;
        float t5n = (mi_z[nidx] - ray.oz) * ray.rdz;
        float t6n = (mx_z[nidx] - ray.oz) * ray.rdz;

        float tminn1 = fminf(t1n, t2n), tmaxn1 = fmaxf(t1n, t2n);
        float tminn2 = fminf(t3n, t4n), tmaxn2 = fmaxf(t3n, t4n);
        float tminn3 = fminf(t5n, t6n), tmaxn3 = fmaxf(t5n, t6n);

        float tminn = fmaxf(fmaxf(tminn1, tminn2), tminn3);
        float tmaxn = fminf(fminf(tmaxn1, tmaxn2), tmaxn3);

        bool nOut = ray.ox < mi_x[nidx] | ray.ox > mx_x[nidx] |
                    ray.oy < mi_y[nidx] | ray.oy > mx_y[nidx] |
                    ray.oz < mi_z[nidx] | ray.oz > mx_z[nidx];
        bool nMiss = tmaxn < tminn | (tminn < 0 & nOut) | tminn > H_t;

        // If node is not a leaf:
        if (!lf[nidx] & !nMiss) {
            // Find the distance to the left child
            int tcl = pl[nidx];
            float t1l = (mi_x[tcl] - ray.ox) * ray.rdx;
            float t2l = (mx_x[tcl] - ray.ox) * ray.rdx;
            float t3l = (mi_y[tcl] - ray.oy) * ray.rdy;
            float t4l = (mx_y[tcl] - ray.oy) * ray.rdy;
            float t5l = (mi_z[tcl] - ray.oz) * ray.rdz;
            float t6l = (mx_z[tcl] - ray.oz) * ray.rdz;

            float tminl1 = fminf(t1l, t2l), tmaxl1 = fmaxf(t1l, t2l);
            float tminl2 = fminf(t3l, t4l), tmaxl2 = fmaxf(t3l, t4l);
            float tminl3 = fminf(t5l, t6l), tmaxl3 = fmaxf(t5l, t6l);

            float tminl = fmaxf(fmaxf(tminl1, tminl2), tminl3);
            float tmaxl = fminf(fminf(tmaxl1, tmaxl2), tmaxl3);

            bool lOut = ray.ox < mi_x[tcl] | ray.ox > mx_x[tcl] |
                        ray.oy < mi_y[tcl] | ray.oy > mx_y[tcl] |
                        ray.oz < mi_z[tcl] | ray.oz > mx_z[tcl];
            bool lMiss = tmaxl < tminl | tminl < 0;
            float lDist = (-lMiss + tminl * !lMiss) * lOut;

            // Find the distance to the right child
            int tcr = pr[nidx];
            float t1r = (mi_x[tcr] - ray.ox) * ray.rdx;
            float t2r = (mx_x[tcr] - ray.ox) * ray.rdx;
            float t3r = (mi_y[tcr] - ray.oy) * ray.rdy;
            float t4r = (mx_y[tcr] - ray.oy) * ray.rdy;
            float t5r = (mi_z[tcr] - ray.oz) * ray.rdz;
            float t6r = (mx_z[tcr] - ray.oz) * ray.rdz;

            float tminr1 = fminf(t1r, t2r), tmaxr1 = fmaxf(t1r, t2r);
            float tminr2 = fminf(t3r, t4r), tmaxr2 = fmaxf(t3r, t4r);
            float tminr3 = fminf(t5r, t6r), tmaxr3 = fmaxf(t5r, t6r);

            float tminr = fmaxf(fmaxf(tminr1, tminr2), tminr3);
            float tmaxr = fminf(fminf(tmaxr1, tmaxr2), tmaxr3);

            bool rOut = ray.ox < mi_x[tcr] | ray.ox > mx_x[tcr] |
                        ray.oy < mi_y[tcr] | ray.oy > mx_y[tcr] |
                        ray.oz < mi_z[tcr] | ray.oz > mx_z[tcr];
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

        for (int i = pl[nidx]; i < pr[nidx]; ++i) {
            int gi = gIdx[i];

            bool hit = true;

            float e1x = vx[fv1[gi]] - vx[fv0[gi]];
            float e1y = vy[fv1[gi]] - vy[fv0[gi]];
            float e1z = vz[fv1[gi]] - vz[fv0[gi]];

            float e2x = vx[fv2[gi]] - vx[fv0[gi]];
            float e2y = vy[fv2[gi]] - vy[fv0[gi]];
            float e2z = vz[fv2[gi]] - vz[fv0[gi]];

            float hx = ray.dy * e2z - ray.dz * e2y;
            float hy = ray.dz * e2x - ray.dx * e2z;
            float hz = ray.dx * e2y - ray.dy * e2x;

            float a = e1x * hx + e1y * hy + e1z * hz;

            hit &= a != 0.0f;
            a = !hit + a;

            float sx = ray.ox - vx[fv0[gi]];
            float sy = ray.oy - vy[fv0[gi]];
            float sz = ray.oz - vz[fv0[gi]];

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

    const AzMtl &H_m = mats[fm[H_Idx]];

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

    // Fake shading
    bool fShade = fakeShading && H_hasN;
    float H_NdotR_D = H_nx * ray.dx + H_ny * ray.dy + H_nz * ray.dz;
    H_NdotR_D = (0.4 + H_NdotR_D * H_NdotR_D * 0.6) * fShade + !fShade;

    frmx[tIdx] = H_alb_x * H_NdotR_D;
    frmy[tIdx] = H_alb_y * H_NdotR_D;
    frmz[tIdx] = H_alb_z * H_NdotR_D;
    frmdepth[tIdx] = H_t;
    frmmat[tIdx] = fm[H_Idx];
};