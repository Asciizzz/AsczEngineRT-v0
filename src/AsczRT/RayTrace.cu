#include <RayTrace.cuh>
#include <AzDevMath.cuh>


__global__ void raytraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats, int *lSrc, int lNum, 
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    float falseAmbient
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 8;
    const int MAX_NODES = 64;

    Ray rstack[MAX_RAYS];
    int rs_top = 0;

    rstack[rs_top++] = primaryRay;

    int nstack[MAX_NODES];
    int ns_top = 0;

    Flt3 resultColr;
    while (rs_top > 0) {
        // Copy before pop since there's high chance of overwriting
        Ray ray = rstack[--rs_top];
        int hidx = -1;
        float ht = 1e9f;
        float hu = 0.0f;
        float hv = 0.0f;

        ns_top = 0;
        nstack[ns_top++] = 0;

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

                bool hit = gi != ray.ignore;

                float e1x = vx[fv1[gi]] - vx[fv0[gi]];
                float e1y = vy[fv1[gi]] - vy[fv0[gi]];
                float e1z = vz[fv1[gi]] - vz[fv0[gi]];

                float e2x = vx[fv2[gi]] - vx[fv0[gi]];
                float e2y = vy[fv2[gi]] - vy[fv0[gi]];
                float e2z = vz[fv2[gi]] - vz[fv0[gi]];

                float hx = ray.d.y * e2z - ray.d.z * e2y;
                float hy = ray.d.z * e2x - ray.d.x * e2z;
                float hz = ray.d.x * e2y - ray.d.y * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = a == 0.0f ? 1.0f : a;

                float f = 1.0f / a;

                float sx = ray.o.x - vx[fv0[gi]];
                float sy = ray.o.y - vy[fv0[gi]];
                float sz = ray.o.z - vz[fv0[gi]];

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (ray.d.x * qx + ray.d.y * qy + ray.d.z * qz);

                hit &= v >= 0.0f & u + v <= 1.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < ht;

                ht = t * hit + ht * !hit;
                hu = u * hit + hu * !hit;
                hv = v * hit + hv * !hit;
                hidx = gi * hit + hidx * !hit;
            }
        }

        if (hidx == -1) continue;

        // Get the face data
        const AzMtl &hm = mats[fm[hidx]];
        float hw = 1.0f - hu - hv;

        // Vertex interpolation
        Flt3 vrtx = ray.o + ray.d * ht;

        // Texture interpolation (if available)
        bool hasTxtr = hm.AlbMap > -1;
        int t0 = ft0[hidx], t1 = ft1[hidx], t2 = ft2[hidx];
        float t_u = hasTxtr ? tx[t0] * hw + tx[t1] * hu + tx[t2] * hv : 0.0f;
        float t_v = hasTxtr ? ty[t0] * hw + ty[t1] * hu + ty[t2] * hv : 0.0f;
        t_u -= floor(t_u); t_v -= floor(t_v);

        int t_w = hasTxtr ? tw[hm.AlbMap] : 0;
        int t_h = hasTxtr ? th[hm.AlbMap] : 0;
        int t_off = hasTxtr ? toff[hm.AlbMap] : 0;

        int t_x = (int)(t_u * t_w);
        int t_y = (int)(t_v * t_h);
        int t_idx = t_off + t_y * t_w + t_x;

        Flt3 alb = hasTxtr ? Flt3(tr[t_idx], tg[t_idx], tb[t_idx]) : hm.Alb;

        // Normal interpolation
        Flt3 nrml; bool hasNrml = fn0[hidx] > -1;
        int n0 = fn0[hidx], n1 = fn1[hidx], n2 = fn2[hidx];
        nrml.x = hasNrml ? nx[n0] * hw + nx[n1] * hu + nx[n2] * hv : 0.0f;
        nrml.y = hasNrml ? ny[n0] * hw + ny[n1] * hu + ny[n2] * hv : 0.0f;
        nrml.z = hasNrml ? nz[n0] * hw + nz[n1] * hu + nz[n2] * hv : 0.0f;

        // Fake ambient
        float NdotL = (nrml * ray.d) * hasNrml + !hasNrml;
        Flt3 finalColr = alb * falseAmbient * NdotL * NdotL;

        // Direct lighting
        for (int l = 0; l < lNum; ++l) {
            // Get material and geometry data of light
            int lIdx = lSrc[l];

            int fl0 = fv0[lIdx], fl1 = fv1[lIdx], fl2 = fv2[lIdx];

            float lpx = (vx[fl0] + vx[fl1] + vx[fl2]) / 3.0f;
            float lpy = (vy[fl0] + vy[fl1] + vy[fl2]) / 3.0f;
            float lpz = (vz[fl0] + vz[fl1] + vz[fl2]) / 3.0f;

            float ldx = vrtx.x - lpx;
            float ldy = vrtx.y - lpy;
            float ldz = vrtx.z - lpz;

            float ldst = sqrtf(ldx * ldx + ldy * ldy + ldz * ldz);

            ldx /= ldst;
            ldy /= ldst;
            ldz /= ldst;

            float linvx = 1.0f / ldx;
            float linvy = 1.0f / ldy;
            float linvz = 1.0f / ldz;

            // Reset the stack
            ns_top = 0;
            nstack[ns_top++] = 0;

            bool inLight = true;
            while (ns_top > 0) {
                int nidx = nstack[--ns_top];

                float t1n = (mi_x[nidx] - lpx) * linvx;
                float t2n = (mx_x[nidx] - lpx) * linvx;
                float t3n = (mi_y[nidx] - lpy) * linvy;
                float t4n = (mx_y[nidx] - lpy) * linvy;
                float t5n = (mi_z[nidx] - lpz) * linvz;
                float t6n = (mx_z[nidx] - lpz) * linvz;

                float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
                tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
                tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

                bool nOut = lpx < mi_x[nidx] | lpx > mx_x[nidx] |
                            lpy < mi_y[nidx] | lpy > mx_y[nidx] |
                            lpz < mi_z[nidx] | lpz > mx_z[nidx];
                float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

                if (nDist < 0 | nDist > ldst) continue;

                if (!lf[nidx]) {
                    int tcl = pl[nidx];
                    float t1l = (mi_x[tcl] - lpx) * linvx;
                    float t2l = (mx_x[tcl] - lpx) * linvx;
                    float t3l = (mi_y[tcl] - lpy) * linvy;
                    float t4l = (mx_y[tcl] - lpy) * linvy;
                    float t5l = (mi_z[tcl] - lpz) * linvz;
                    float t6l = (mx_z[tcl] - lpz) * linvz;

                    float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                    tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                    tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                    bool lOut = lpx < mi_x[tcl] | lpx > mx_x[tcl] |
                                lpy < mi_y[tcl] | lpy > mx_y[tcl] |
                                lpz < mi_z[tcl] | lpz > mx_z[tcl];
                    float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;


                    int tcr = pr[nidx];
                    float t1r = (mi_x[tcr] - lpx) * linvx;
                    float t2r = (mx_x[tcr] - lpx) * linvx;
                    float t3r = (mi_y[tcr] - lpy) * linvy;
                    float t4r = (mx_y[tcr] - lpy) * linvy;
                    float t5r = (mi_z[tcr] - lpz) * linvz;
                    float t6r = (mx_z[tcr] - lpz) * linvz;

                    float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                    tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                    tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                    bool rOut = lpx < mi_x[tcr] | lpx > mx_x[tcr] |
                                lpy < mi_y[tcr] | lpy > mx_y[tcr] |
                                lpz < mi_z[tcr] | lpz > mx_z[tcr];
                    float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


                    nstack[ns_top] = tcl;
                    ns_top += (ldist >= 0);

                    nstack[ns_top] = tcr;
                    ns_top += (rdist >= 0);
    
                    continue;
                }
    
                for (int i = pl[nidx]; i < pr[nidx]; ++i) {
                    int gi = gIdx[i];

                    bool hit = gi != hidx & gi != lIdx;

                    int f0 = fv0[gi], f1 = fv1[gi], f2 = fv2[gi];

                    float e1x = vx[f1] - vx[f0];
                    float e1y = vy[f1] - vy[f0];
                    float e1z = vz[f1] - vz[f0];

                    float e2x = vx[f2] - vx[f0];
                    float e2y = vy[f2] - vy[f0];
                    float e2z = vz[f2] - vz[f0];

                    float hx = ldy * e2z - ldz * e2y;
                    float hy = ldz * e2x - ldx * e2z;
                    float hz = ldx * e2y - ldy * e2x;

                    float a = e1x * hx + e1y * hy + e1z * hz;

                    hit &= a != 0;
                    a = a == 0 ? 1 : a;

                    float f = 1.0f / a;

                    float sx = lpx - vx[fv0[gi]];
                    float sy = lpy - vy[fv0[gi]];
                    float sz = lpz - vz[fv0[gi]];

                    float u = f * (sx * hx + sy * hy + sz * hz);

                    hit &= u >= 0 & u <= 1;

                    float qx = sy * e1z - sz * e1y;
                    float qy = sz * e1x - sx * e1z;
                    float qz = sx * e1y - sy * e1x;

                    float v = f * (ldx * qx + ldy * qy + ldz * qz);

                    hit &= v >= 0 & u + v <= 1;

                    float t = f * (e2x * qx + e2y * qy + e2z * qz);

                    hit &= t > 0 & t < ldst;

                    inLight &= !hit;
                    ns_top *= inLight;
                }
            }

            bool angular = hasNrml && !hm.NoShade;
            float NdotL = -(nrml.x * ldx + nrml.y * ldy + nrml.z * ldz);
            Flt3 diff = alb * (NdotL * (NdotL > 0.0f) * angular + !angular);
        }

        // ======== Additional rays ========

        // Transmission ray
        float trLeft = ray.w * hm.Tr;
        ray.w *= (1 - hm.Tr);

        Flt3 trO = vrtx + ray.d * EPSILON_1;
        rstack[rs_top] = Ray(trO, ray.d, trLeft, hm.Ior, hidx);
        rs_top += rs_top + 1 < MAX_RAYS & hm.Tr > 0.0f;

        // Reflection ray
        float rfLeft = ray.w * hm.Rf;
        ray.w *= (1 - hm.Rf);

        Flt3 rfD = ray.d - nrml * 2.0f * (nrml * ray.d);
        Flt3 rfO = vrtx + nrml * EPSILON_1;
        rstack[rs_top] = Ray(rfO, rfD, rfLeft, hm.Ior, hidx);
        rs_top += rs_top + 1 < MAX_RAYS & hm.Rf > 0.0f;


        // Accumulate the result
        resultColr += finalColr * ray.w;
    }

    frmbuffer[tIdx] = resultColr;
}
