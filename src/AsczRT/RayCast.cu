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

        float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
        tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
        tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

        bool nOut = ray.ox < mi_x[nidx] | ray.ox > mx_x[nidx] |
                    ray.oy < mi_y[nidx] | ray.oy > mx_y[nidx] |
                    ray.oz < mi_z[nidx] | ray.oz > mx_z[nidx];
        float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

        if (nDist < 0 | nDist > H_t) continue;

        // If node is not a leaf:
        if (!lf[nidx]) {
            // Find the distance to the left child
            int tcl = pl[nidx];
            float t1l = (mi_x[tcl] - ray.ox) * ray.rdx;
            float t2l = (mx_x[tcl] - ray.ox) * ray.rdx;
            float t3l = (mi_y[tcl] - ray.oy) * ray.rdy;
            float t4l = (mx_y[tcl] - ray.oy) * ray.rdy;
            float t5l = (mi_z[tcl] - ray.oz) * ray.rdz;
            float t6l = (mx_z[tcl] - ray.oz) * ray.rdz;

            float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
            tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
            tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

            bool lOut = ray.ox < mi_x[tcl] | ray.ox > mx_x[tcl] |
                        ray.oy < mi_y[tcl] | ray.oy > mx_y[tcl] |
                        ray.oz < mi_z[tcl] | ray.oz > mx_z[tcl];
            float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

            // Find the distance to the rigH_t child
            int tcr = pr[nidx];
            float t1r = (mi_x[tcr] - ray.ox) * ray.rdx;
            float t2r = (mx_x[tcr] - ray.ox) * ray.rdx;
            float t3r = (mi_y[tcr] - ray.oy) * ray.rdy;
            float t4r = (mx_y[tcr] - ray.oy) * ray.rdy;
            float t5r = (mi_z[tcr] - ray.oz) * ray.rdz;
            float t6r = (mx_z[tcr] - ray.oz) * ray.rdz;

            float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
            tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
            tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

            bool rOut = ray.ox < mi_x[tcr] | ray.ox > mx_x[tcr] |
                        ray.oy < mi_y[tcr] | ray.oy > mx_y[tcr] |
                        ray.oz < mi_z[tcr] | ray.oz > mx_z[tcr];
            float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


            // Child ordering for closer intersection and early exit
            bool lcloser = ldist < rdist;

            nstack[ns_top] = tcr * lcloser + tcl * !lcloser;
            ns_top += (rdist >= 0) * lcloser + (ldist >= 0) * !lcloser;

            nstack[ns_top] = tcl * lcloser + tcr * !lcloser;
            ns_top += (ldist >= 0) * lcloser + (rdist >= 0) * !lcloser;

            continue;
        }

        for (int i = pl[nidx]; i < pr[nidx]; ++i) {
            int gi = gIdx[i];

            bool hit = true;

            int f0 = fv0[gi];
            int f1 = fv1[gi];
            int f2 = fv2[gi];

            float e1x = vx[f1] - vx[f0];
            float e1y = vy[f1] - vy[f0];
            float e1z = vz[f1] - vz[f0];

            float e2x = vx[f2] - vx[f0];
            float e2y = vy[f2] - vy[f0];
            float e2z = vz[f2] - vz[f0];

            float hx = ray.dy * e2z - ray.dz * e2y;
            float hy = ray.dz * e2x - ray.dx * e2z;
            float hz = ray.dx * e2y - ray.dy * e2x;

            float a = e1x * hx + e1y * hy + e1z * hz;

            hit &= a != 0.0f;
            a = a == 0.0f ? 1.0f : a;

            float f = 1.0f / a;

            float sx = ray.ox - vx[f0];
            float sy = ray.oy - vy[f0];
            float sz = ray.oz - vz[f0];

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
        frmx[tIdx] = 0.0f;
        frmy[tIdx] = 0.0f;
        frmz[tIdx] = 0.0f;
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
    float NdotL = H_nx * ray.dx + H_ny * ray.dy + H_nz * ray.dz;
    NdotL = NdotL * NdotL * fShade + !fShade;

    frmx[tIdx] = H_alb_x * NdotL;
    frmy[tIdx] = H_alb_y * NdotL;
    frmz[tIdx] = H_alb_z * NdotL;
    frmdepth[tIdx] = H_t;
    frmmat[tIdx] = fm[H_Idx];
};