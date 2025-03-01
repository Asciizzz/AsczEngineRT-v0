#include <RayTrace.cuh>

#include <curand_kernel.h>

struct RayHit {
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

__device__ Flt4 getTextureColor(
    float u, float v,
    float *tr, float *tg, float *tb, float *ta,
    int *tw, int *th, int *toff, int AlbMap
) {
    u -= floor(u);
    v -= floor(v);

    int w = tw[AlbMap];
    int h = th[AlbMap];
    int off = toff[AlbMap];

    int tx = (int)(u * w);
    int ty = (int)(v * h);

    int t = off + ty * w + tx;
    return Flt4(tr[t], tg[t], tb[t], ta[t]);
}

__device__ Flt3 ASESFilm(const Flt3 &P) {
    const float a = 2.51f;
    const float b = 0.03f;
    const float c = 2.43f;
    const float d = 0.59f;
    const float e = 0.14f;

    Flt3 y = Flt3(
        (P.x * (a * P.x + b)) / (P.x * (c * P.x + d) + e),
        (P.y * (a * P.y + b)) / (P.y * (c * P.y + d) + e),
        (P.z * (a * P.z + b)) / (P.z * (c * P.z + d) + e)
    ).clamp(0.0f, 1.0f);

    return y;
}

__device__ Flt3 randomHemisphereSample(curandState *rnd, const Flt3 &n) {
    float r1 = curand_uniform(rnd);  // Random number [0,1]
    float r2 = curand_uniform(rnd);

    float theta = acos(sqrt(1.0f - r1));  // Importance sampling (cosine-weighted)
    float phi = 2.0f * M_PI * r2;         // Uniform azimuthal angle

    // Convert to Cartesian coordinates
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);

    // Construct a coordinate system
    Flt3 tangent, bitangent;

    tangent = n.x > 0.9f || n.x < -0.9f ? Flt3(0, 1, 0) : Flt3(1, 0, 0);
    tangent = (tangent ^ n).norm();
    bitangent = n ^ tangent;

    // Transform to world space
    return tangent * x + bitangent * y + n * z;
}



__global__ void raytraceKernel(
    AsczCam camera, unsigned int *frmbuffer, int frmW, int frmH, // In-out
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Materials
    AzMtl *mats,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Light data
    int *lSrc, int lNum, 
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *cl, int *cr, int *ll, int *lr, int *gIdx,
    // Additional Debug Data
    bool falseAmbient
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 8;
    const int MAX_NODES = 64;

    Ray rstack[MAX_RAYS] = { primaryRay };
    int rs_top = 1;

    int nstack[MAX_NODES];
    int ns_top = 0;

    Flt3 resultColr;
    while (rs_top > 0) {
        // Copy before pop since there's high chance of overwriting
        Ray ray = rstack[--rs_top];
        RayHit rhit;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            float t1n = (mi_x[nidx] - ray.o.x) * ray.invd.x;
            float t2n = (mx_x[nidx] - ray.o.x) * ray.invd.x;
            float t3n = (mi_y[nidx] - ray.o.y) * ray.invd.y;
            float t4n = (mx_y[nidx] - ray.o.y) * ray.invd.y;
            float t5n = (mi_z[nidx] - ray.o.z) * ray.invd.z;
            float t6n = (mx_z[nidx] - ray.o.z) * ray.invd.z;

            float tminn = fmaxf(fmaxf(fminf(t1n, t2n), fminf(t3n, t4n)), fminf(t5n, t6n));
            float tmaxn = fminf(fminf(fmaxf(t1n, t2n), fmaxf(t3n, t4n)), fmaxf(t5n, t6n));

            bool nOut = ray.o.x < mi_x[nidx] | ray.o.x > mx_x[nidx] |
                        ray.o.y < mi_y[nidx] | ray.o.y > mx_y[nidx] |
                        ray.o.z < mi_z[nidx] | ray.o.z > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 || nDist > rhit.t) continue;

            if (cl[nidx] > -1) {
                float t1l = (mi_x[cl[nidx]] - ray.o.x) * ray.invd.x;
                float t2l = (mx_x[cl[nidx]] - ray.o.x) * ray.invd.x;
                float t3l = (mi_y[cl[nidx]] - ray.o.y) * ray.invd.y;
                float t4l = (mx_y[cl[nidx]] - ray.o.y) * ray.invd.y;
                float t5l = (mi_z[cl[nidx]] - ray.o.z) * ray.invd.z;
                float t6l = (mx_z[cl[nidx]] - ray.o.z) * ray.invd.z;

                float tminl = fmaxf(fmaxf(fminf(t1l, t2l), fminf(t3l, t4l)), fminf(t5l, t6l));
                float tmaxl = fminf(fminf(fmaxf(t1l, t2l), fmaxf(t3l, t4l)), fmaxf(t5l, t6l));

                bool lOut = ray.o.x < mi_x[cl[nidx]] | ray.o.x > mx_x[cl[nidx]] |
                            ray.o.y < mi_y[cl[nidx]] | ray.o.y > mx_y[cl[nidx]] |
                            ray.o.z < mi_z[cl[nidx]] | ray.o.z > mx_z[cl[nidx]];
                float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;


                float t1r = (mi_x[cr[nidx]] - ray.o.x) * ray.invd.x;
                float t2r = (mx_x[cr[nidx]] - ray.o.x) * ray.invd.x;
                float t3r = (mi_y[cr[nidx]] - ray.o.y) * ray.invd.y;
                float t4r = (mx_y[cr[nidx]] - ray.o.y) * ray.invd.y;
                float t5r = (mi_z[cr[nidx]] - ray.o.z) * ray.invd.z;
                float t6r = (mx_z[cr[nidx]] - ray.o.z) * ray.invd.z;

                float tminr = fmaxf(fmaxf(fminf(t1r, t2r), fminf(t3r, t4r)), fminf(t5r, t6r));
                float tmaxr = fminf(fminf(fmaxf(t1r, t2r), fmaxf(t3r, t4r)), fmaxf(t5r, t6r));

                bool rOut = ray.o.x < mi_x[cr[nidx]] | ray.o.x > mx_x[cr[nidx]] |
                            ray.o.y < mi_y[cr[nidx]] | ray.o.y > mx_y[cr[nidx]] |
                            ray.o.z < mi_z[cr[nidx]] | ray.o.z > mx_z[cr[nidx]];
                float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;

                bool lcloser = ldist < rdist;

                nstack[ns_top] = cr[nidx] * lcloser + cl[nidx] * !lcloser;
                ns_top += (rdist >= 0) * lcloser + (ldist >= 0) * !lcloser;

                nstack[ns_top] = cl[nidx] * lcloser + cr[nidx] * !lcloser;
                ns_top += (ldist >= 0) * lcloser + (rdist >= 0) * !lcloser;

                continue;
            }

            for (int i = ll[nidx]; i < lr[nidx]; ++i) {
                int gi = gIdx[i];

                bool hit = gi != rhit.idx;

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

                hit &= u >= 0.0f && u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (ray.d.x * qx + ray.d.y * qy + ray.d.z * qz);

                hit &= v >= 0.0f && u + v <= 1.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f && t < rhit.t;

                rhit.t = t * hit + rhit.t * !hit;
                rhit.u = u * hit + rhit.u * !hit;
                rhit.v = v * hit + rhit.v * !hit;
                rhit.idx = gi * hit + rhit.idx * !hit;
            }
        }

        int hIdx = rhit.idx;
        if (hIdx == -1) continue;

        // Get the face data
        const AzMtl &hm = mats[fm[hIdx]];

        float rhitw = 1 - rhit.u - rhit.v;

        Flt3 vrtx = ray.o + ray.d * rhit.t;

        Flt3 nrml = {
            nx[fn0[hIdx]] * rhitw + nx[fn1[hIdx]] * rhit.u + nx[fn2[hIdx]] * rhit.v,
            ny[fn0[hIdx]] * rhitw + ny[fn1[hIdx]] * rhit.u + ny[fn2[hIdx]] * rhit.v,
            nz[fn0[hIdx]] * rhitw + nz[fn1[hIdx]] * rhit.u + nz[fn2[hIdx]] * rhit.v
        };

        Flt3 alb;
        if (hm.AlbMap > -1) {
            Int3 tt = Int3(ft0[hIdx], ft1[hIdx], ft2[hIdx]);
            float tu = tx[tt.x] * rhitw + tx[tt.y] * rhit.u + tx[tt.z] * rhit.v;
            float tv = ty[tt.x] * rhitw + ty[tt.y] * rhit.u + ty[tt.z] * rhit.v;

            Flt4 txColr = getTextureColor(tu, tv, tr, tg, tb, ta, tw, th, toff, hm.AlbMap);
            alb = txColr.f3();
        } else {
            alb = hm.Alb;
        }

        // Lighting and shading
        float NdotL = falseAmbient ? nrml * ray.d : 0.0f;
        Flt3 finalColr = alb * 0.02f * NdotL * NdotL;

        if (!hm.Ems.isZero()) {
            resultColr += hm.Ems * ray.w;
            continue;
        }

        // Direct lighting
        for (int l = 0; l < lNum; ++l) {
            // Get material and geometry data of light
            int lIdx = lSrc[l];
            const AzMtl &lMat = mats[fm[lIdx]];

            // Get position based on the geometry type
            Flt3 lPos = {
                vx[fv0[lIdx]] + vx[fv1[lIdx]] + vx[fv2[lIdx]],
                vy[fv0[lIdx]] + vy[fv1[lIdx]] + vy[fv2[lIdx]],
                vz[fv0[lIdx]] + vz[fv1[lIdx]] + vz[fv2[lIdx]]
            };
            lPos /= 3.0f;

            Flt3 lDir = vrtx - lPos;
            float lDist = lDir.mag();
            if (lDist < 0.01f) continue;

            lDir /= lDist;
            Flt3 lInv = 1.0f / lDir;

            // Reset the stack
            ns_top = 0;
            nstack[ns_top++] = 0;

            bool shadow = false;
            while (ns_top > 0) {
                int nidx = nstack[--ns_top];

                float t1n = (mi_x[nidx] - lPos.x) * lInv.x;
                float t2n = (mx_x[nidx] - lPos.x) * lInv.x;
                float t3n = (mi_y[nidx] - lPos.y) * lInv.y;
                float t4n = (mx_y[nidx] - lPos.y) * lInv.y;
                float t5n = (mi_z[nidx] - lPos.z) * lInv.z;
                float t6n = (mx_z[nidx] - lPos.z) * lInv.z;

                float tminn = fmaxf(fmaxf(fminf(t1n, t2n), fminf(t3n, t4n)), fminf(t5n, t6n));
                float tmaxn = fminf(fminf(fmaxf(t1n, t2n), fmaxf(t3n, t4n)), fmaxf(t5n, t6n));
                
                bool nOut = lPos.x < mi_x[nidx] | lPos.x > mx_x[nidx] |
                            lPos.y < mi_y[nidx] | lPos.y > mx_y[nidx] |
                            lPos.z < mi_z[nidx] | lPos.z > mx_z[nidx];
                float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

                if (nDist < 0 || nDist > lDist) continue;

                if (cl[nidx] > -1) {
                    float t1l = (mi_x[cl[nidx]] - lPos.x) * lInv.x;
                    float t2l = (mx_x[cl[nidx]] - lPos.x) * lInv.x;
                    float t3l = (mi_y[cl[nidx]] - lPos.y) * lInv.y;
                    float t4l = (mx_y[cl[nidx]] - lPos.y) * lInv.y;
                    float t5l = (mi_z[cl[nidx]] - lPos.z) * lInv.z;
                    float t6l = (mx_z[cl[nidx]] - lPos.z) * lInv.z;

                    float tminl = fmaxf(fmaxf(fminf(t1l, t2l), fminf(t3l, t4l)), fminf(t5l, t6l));
                    float tmaxl = fminf(fminf(fmaxf(t1l, t2l), fmaxf(t3l, t4l)), fmaxf(t5l, t6l));

                    bool lOut = lPos.x < mi_x[cl[nidx]] | lPos.x > mx_x[cl[nidx]] |
                                lPos.y < mi_y[cl[nidx]] | lPos.y > mx_y[cl[nidx]] |
                                lPos.z < mi_z[cl[nidx]] | lPos.z > mx_z[cl[nidx]];
                    float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;


                    float t1r = (mi_x[cr[nidx]] - lPos.x) * lInv.x;
                    float t2r = (mx_x[cr[nidx]] - lPos.x) * lInv.x;
                    float t3r = (mi_y[cr[nidx]] - lPos.y) * lInv.y;
                    float t4r = (mx_y[cr[nidx]] - lPos.y) * lInv.y;
                    float t5r = (mi_z[cr[nidx]] - lPos.z) * lInv.z;
                    float t6r = (mx_z[cr[nidx]] - lPos.z) * lInv.z;

                    float tminr = fmaxf(fmaxf(fminf(t1r, t2r), fminf(t3r, t4r)), fminf(t5r, t6r));
                    float tmaxr = fminf(fminf(fmaxf(t1r, t2r), fmaxf(t3r, t4r)), fmaxf(t5r, t6r));

                    bool rOut = lPos.x < mi_x[cr[nidx]] | lPos.x > mx_x[cr[nidx]] |
                                lPos.y < mi_y[cr[nidx]] | lPos.y > mx_y[cr[nidx]] |
                                lPos.z < mi_z[cr[nidx]] | lPos.z > mx_z[cr[nidx]];
                    float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


                    nstack[ns_top] = cl[nidx];
                    ns_top += (ldist >= 0);

                    nstack[ns_top] = cr[nidx];
                    ns_top += (rdist >= 0);
    
                    continue;
                }
    
                for (int i = ll[nidx]; i < lr[nidx]; ++i) {
                    int gi = gIdx[i];

                    bool hit = gi != hIdx && gi != lIdx;

                    float e1x = vx[fv1[gi]] - vx[fv0[gi]];
                    float e1y = vy[fv1[gi]] - vy[fv0[gi]];
                    float e1z = vz[fv1[gi]] - vz[fv0[gi]];

                    float e2x = vx[fv2[gi]] - vx[fv0[gi]];
                    float e2y = vy[fv2[gi]] - vy[fv0[gi]];
                    float e2z = vz[fv2[gi]] - vz[fv0[gi]];

                    float hx = lDir.y * e2z - lDir.z * e2y;
                    float hy = lDir.z * e2x - lDir.x * e2z;
                    float hz = lDir.x * e2y - lDir.y * e2x;

                    float a = e1x * hx + e1y * hy + e1z * hz;

                    hit &= a != 0;
                    a = a == 0 ? 1 : a;

                    float f = 1.0f / a;

                    float sx = lPos.x - vx[fv0[gi]];
                    float sy = lPos.y - vy[fv0[gi]];
                    float sz = lPos.z - vz[fv0[gi]];

                    float u = f * (sx * hx + sy * hy + sz * hz);

                    hit &= u >= 0 && u <= 1;

                    float qx = sy * e1z - sz * e1y;
                    float qy = sz * e1x - sx * e1z;
                    float qz = sx * e1y - sy * e1x;

                    float v = f * (lDir.x * qx + lDir.y * qy + lDir.z * qz);

                    hit &= v >= 0 && u + v <= 1;

                    float t = f * (e2x * qx + e2y * qy + e2z * qz);

                    hit &= t > 0 && t < lDist;

                    shadow |= hit;
                    ns_top *= !shadow;
                }
            }

            float NdotL = nrml * -lDir;
            NdotL = NdotL < 0 ? 0 : NdotL;
            Flt3 diff = alb * NdotL;

            finalColr += shadow ? 0 : lMat.Ems & diff;
        }

        // ======== Additional rays ========

        // Transparent
        if (hm.Tr > 0.0f && rs_top + 2 < MAX_RAYS) {
            float wLeft = ray.w * hm.Tr;
            ray.w *= (1 - hm.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;
            rstack[rs_top++] = Ray(rO, ray.d, wLeft, hm.Ior, hIdx);
        }

        resultColr += finalColr * ray.w;
    }

    // Tone mapping
    resultColr = ASESFilm(resultColr);

    float _gamma = 1.0f / 2.2f;
    resultColr = resultColr.pow(_gamma);

    int r = (int)(resultColr.x * 255);
    int g = (int)(resultColr.y * 255);
    int b = (int)(resultColr.z * 255);

    frmbuffer[tIdx] = (r << 16) | (g << 8) | b;
}
