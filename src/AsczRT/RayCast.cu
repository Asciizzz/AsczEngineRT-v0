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
    int hidx = -1;
    float ht = 1e9f;
    float hu = 0.0f;
    float hv = 0.0f;
    float hw = 0.0f;

    const int MAX_NODES = 64;
    int nstack[MAX_NODES] = { 0 };
    int ns_top = 1;

    while (ns_top > 0) {
        int nidx = nstack[--ns_top];

        // Check if the ray is outside the bounding box
        float t1n = (mi_x[nidx] - ray.o.x) * ray.invd.x;
        float t2n = (mx_x[nidx] - ray.o.x) * ray.invd.x;
        float t3n = (mi_y[nidx] - ray.o.y) * ray.invd.y;
        float t4n = (mx_y[nidx] - ray.o.y) * ray.invd.y;
        float t5n = (mi_z[nidx] - ray.o.z) * ray.invd.z;
        float t6n = (mx_z[nidx] - ray.o.z) * ray.invd.z;

        float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
        tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
        tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

        bool nOut = ray.o.x < mi_x[nidx] | ray.o.x > mx_x[nidx] |
                    ray.o.y < mi_y[nidx] | ray.o.y > mx_y[nidx] |
                    ray.o.z < mi_z[nidx] | ray.o.z > mx_z[nidx];
        float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

        if (nDist < 0 | nDist > ht) continue;

        // If node is not a leaf:
        if (!lf[nidx]) {
            // Find the distance to the left child
            int tcl = pl[nidx];
            float t1l = (mi_x[tcl] - ray.o.x) * ray.invd.x;
            float t2l = (mx_x[tcl] - ray.o.x) * ray.invd.x;
            float t3l = (mi_y[tcl] - ray.o.y) * ray.invd.y;
            float t4l = (mx_y[tcl] - ray.o.y) * ray.invd.y;
            float t5l = (mi_z[tcl] - ray.o.z) * ray.invd.z;
            float t6l = (mx_z[tcl] - ray.o.z) * ray.invd.z;

            float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
            tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
            tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

            bool lOut = ray.o.x < mi_x[tcl] | ray.o.x > mx_x[tcl] |
                        ray.o.y < mi_y[tcl] | ray.o.y > mx_y[tcl] |
                        ray.o.z < mi_z[tcl] | ray.o.z > mx_z[tcl];
            float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

            // Find the distance to the right child
            int tcr = pr[nidx];
            float t1r = (mi_x[tcr] - ray.o.x) * ray.invd.x;
            float t2r = (mx_x[tcr] - ray.o.x) * ray.invd.x;
            float t3r = (mi_y[tcr] - ray.o.y) * ray.invd.y;
            float t4r = (mx_y[tcr] - ray.o.y) * ray.invd.y;
            float t5r = (mi_z[tcr] - ray.o.z) * ray.invd.z;
            float t6r = (mx_z[tcr] - ray.o.z) * ray.invd.z;

            float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
            tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
            tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

            bool rOut = ray.o.x < mi_x[tcr] | ray.o.x > mx_x[tcr] |
                        ray.o.y < mi_y[tcr] | ray.o.y > mx_y[tcr] |
                        ray.o.z < mi_z[tcr] | ray.o.z > mx_z[tcr];
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

            float hx = ray.d.y * e2z - ray.d.z * e2y;
            float hy = ray.d.z * e2x - ray.d.x * e2z;
            float hz = ray.d.x * e2y - ray.d.y * e2x;

            float a = e1x * hx + e1y * hy + e1z * hz;

            hit &= a != 0.0f;
            a = a == 0.0f ? 1.0f : a;

            float f = 1.0f / a;

            float sx = ray.o.x - vx[f0];
            float sy = ray.o.y - vy[f0];
            float sz = ray.o.z - vz[f0];

            float u = f * (sx * hx + sy * hy + sz * hz);

            hit &= u >= 0.0f & u <= 1.0f;

            float qx = sy * e1z - sz * e1y;
            float qy = sz * e1x - sx * e1z;
            float qz = sx * e1y - sy * e1x;

            float v = f * (ray.d.x * qx + ray.d.y * qy + ray.d.z * qz);
            
            float w = 1.0f - u - v;

            hit &= v >= 0.0f & w >= 0.0f;

            float t = f * (e2x * qx + e2y * qy + e2z * qz);

            hit &= t > 0.0f & t < ht;

            ht = t * hit + ht * !hit;
            hu = u * hit + hu * !hit;
            hv = v * hit + hv * !hit;
            hw = w * hit + hw * !hit;
            hidx = gi * hit + hidx * !hit;
        }
    }

    if (hidx == -1) {
        frmx[tIdx] = 0.0f;
        frmy[tIdx] = 0.0f;
        frmz[tIdx] = 0.0f;
        frmdepth[tIdx] = -1.0f;
        frmmat[tIdx] = -1;
        return;
    }

    const AzMtl &hm = mats[fm[hidx]];

    // Normal interpolation
    int n0 = fn0[hidx], n1 = fn1[hidx], n2 = fn2[hidx];
    float nrml_x = nx[n0] * hw + nx[n1] * hu + nx[n2] * hv;
    float nrml_y = ny[n0] * hw + ny[n1] * hu + ny[n2] * hv;
    float nrml_z = nz[n0] * hw + nz[n1] * hu + nz[n2] * hv;
    bool hasNrml = n0 > 0;

    // Texture interpolation (if available)
    int t0 = ft0[hidx], t1 = ft1[hidx], t2 = ft2[hidx];
    float t_u = tx[t0] * hw + tx[t1] * hu + tx[t2] * hv;
    float t_v = ty[t0] * hw + ty[t1] * hu + ty[t2] * hv;
    t_u -= floor(t_u); t_v -= floor(t_v);

    int alb_map = hm.AlbMap;
    int t_w = tw[alb_map];
    int t_h = th[alb_map];
    int t_off = toff[alb_map];

    int t_x = (int)(t_u * t_w);
    int t_y = (int)(t_v * t_h);
    int t_idx = t_off + t_y * t_w + t_x;

    bool hasTxtr = hm.AlbMap > 0;
    float alb_x = tr[t_idx] * hasTxtr + hm.Alb_r * !hasTxtr;
    float alb_y = tg[t_idx] * hasTxtr + hm.Alb_g * !hasTxtr;
    float alb_z = tb[t_idx] * hasTxtr + hm.Alb_b * !hasTxtr;

    // Fake shading
    bool fShade = fakeShading && hasNrml;
    float NdotL = nrml_x * ray.d.x + nrml_y * ray.d.y + nrml_z * ray.d.z;
    NdotL *= NdotL;

    alb_x *= NdotL * fShade + !fShade;
    alb_y *= NdotL * fShade + !fShade;
    alb_z *= NdotL * fShade + !fShade;

    frmx[tIdx] = alb_x;
    frmy[tIdx] = alb_y;
    frmz[tIdx] = alb_z;
    frmdepth[tIdx] = ht;
    frmmat[tIdx] = fm[hidx];
};