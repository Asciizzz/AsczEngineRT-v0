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

// Ray intersection

__device__ Flt3 RayHitTriangle(const Flt3 &o, const Flt3 &d, const Flt3 &v0, const Flt3 &v1, const Flt3 &v2) {
    Flt3 e1 = v1 - v0;
    Flt3 e2 = v2 - v0;
    Flt3 h = d ^ e2;
    float a = e1 * h;

    if (a == 0) return -1.0f;

    float f = 1.0f / a;
    Flt3 s = o - v0;
    float u = f * (s * h);

    if (u < 0.0f || u > 1.0f) return -1.0f;

    Flt3 q = s ^ e1;
    float v = f * (d * q);

    if (v < 0.0f || u + v > 1.0f) return -1.0f;

    float t = f * (e2 * q);

    return Flt3(t, u, v);
}

__device__ Flt3 RayHitTriangle(const Flt3 &o, const Flt3 &d, const float vx[3], const float vy[3], const float vz[3]) {
    float e1x = vx[1] - vx[0];
    float e1y = vy[1] - vy[0];
    float e1z = vz[1] - vz[0];

    float e2x = vx[2] - vx[0];
    float e2y = vy[2] - vy[0];
    float e2z = vz[2] - vz[0];

    float hx = d.y * e2z - d.z * e2y;
    float hy = d.z * e2x - d.x * e2z;
    float hz = d.x * e2y - d.y * e2x;

    float a = e1x * hx + e1y * hy + e1z * hz;

    if (a == 0) return -1.0f;

    float f = 1.0f / a;

    float sx = o.x - vx[0];
    float sy = o.y - vy[0];
    float sz = o.z - vz[0];

    float u = f * (sx * hx + sy * hy + sz * hz);

    if (u < 0.0f || u > 1.0f) return -1.0f;

    float qx = sy * e1z - sz * e1y;
    float qy = sz * e1x - sx * e1z;
    float qz = sx * e1y - sy * e1x;

    float v = f * (d.x * qx + d.y * qy + d.z * qz);

    if (v < 0.0f || u + v > 1.0f) return -1.0f;

    float t = f * (e2x * qx + e2y * qy + e2z * qz);

    return Flt3(t, u, v);
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
    int *gIdx, DevNode *nodes, int nNum, 
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
        RayHit hit;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];
            DevNode &node = nodes[nidx];

            float hitDist = node.hitDist(ray.o, ray.invd);
            if (hitDist < 0 || hitDist > hit.t) continue;

            if (node.cl > -1) { // If node not a leaf
                float ldist = nodes[node.cl].hitDist(ray.o, ray.invd);
                float rdist = nodes[node.cr].hitDist(ray.o, ray.invd);

                // Early exit
                if (ldist < 0 && rdist < 0) continue;
                // Push the valid node
                else if (ldist < 0) nstack[ns_top++] = node.cr;
                else if (rdist < 0) nstack[ns_top++] = node.cl;
                // Push the closest node first
                else {
                    nstack[ns_top++] = ldist < rdist ? node.cr : node.cl;
                    nstack[ns_top++] = ldist < rdist ? node.cl : node.cr;
                }

                continue;
            }

            for (int i = node.ll; i < node.lr; ++i) {
                int gi = gIdx[i];
                if (gi == ray.ignore) continue;

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

                if (a == 0) continue;

                float f = 1.0f / a;

                float sx = ray.o.x - vx[fv0[gi]];
                float sy = ray.o.y - vy[fv0[gi]];
                float sz = ray.o.z - vz[fv0[gi]];

                float u = f * (sx * hx + sy * hy + sz * hz);

                if (u < 0.0f || u > 1.0f) continue;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (ray.d.x * qx + ray.d.y * qy + ray.d.z * qz);

                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                if (t > 0 && t < hit.t) {
                    hit.t = t;
                    hit.u = u;
                    hit.v = v;
                    hit.idx = gi;
                }
            }
        }

        int hIdx = hit.idx;
        if (hIdx == -1) continue;

        // Get the face data
        const AzMtl &hm = mats[fm[hIdx]];

        float hitw = 1 - hit.u - hit.v;

        Flt3 vrtx = ray.o + ray.d * hit.t;

        Flt3 nrml;
        nrml.x = nx[fn0[hIdx]] * hitw + nx[fn1[hIdx]] * hit.u + nx[fn2[hIdx]] * hit.v;
        nrml.y = ny[fn0[hIdx]] * hitw + ny[fn1[hIdx]] * hit.u + ny[fn2[hIdx]] * hit.v;
        nrml.z = nz[fn0[hIdx]] * hitw + nz[fn1[hIdx]] * hit.u + nz[fn2[hIdx]] * hit.v;

        Flt3 alb;
        if (hm.AlbMap > -1) {
            Int3 tt = Int3(ft0[hIdx], ft1[hIdx], ft2[hIdx]);
            float u = tx[tt.x] * hitw + tx[tt.y] * hit.u + tx[tt.z] * hit.v;
            float v = ty[tt.x] * hitw + ty[tt.y] * hit.u + ty[tt.z] * hit.v;

            Flt4 txColr = getTextureColor(u, v, tr, tg, tb, ta, tw, th, toff, hm.AlbMap);
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
                int idx = nstack[--ns_top];
                DevNode &node = nodes[idx];

                float hitDist = node.hitDist(lPos, lInv);
                if (hitDist < 0 || hitDist > lDist) continue;

                if (node.cl > -1) { // If node not a leaf
                    float ldist = nodes[node.cl].hitDist(lPos, lInv);
                    float rdist = nodes[node.cr].hitDist(lPos, lInv);

                    if (ldist < 0 && rdist < 0) continue;
                    else if (ldist < 0) nstack[ns_top++] = node.cr;
                    else if (rdist < 0) nstack[ns_top++] = node.cl;
                    else {
                        nstack[ns_top++] = ldist < rdist ? node.cr : node.cl;
                        nstack[ns_top++] = ldist < rdist ? node.cl : node.cr;
                    }

                    continue;
                }

                for (int i = node.ll; i < node.lr; ++i) {
                    int gi = gIdx[i];
                    if (gi == hIdx || gi == lIdx) continue;

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

                    if (a == 0) continue;

                    float f = 1.0f / a;

                    float sx = lPos.x - vx[fv0[gi]];
                    float sy = lPos.y - vy[fv0[gi]];
                    float sz = lPos.z - vz[fv0[gi]];

                    float u = f * (sx * hx + sy * hy + sz * hz);
                    
                    if (u < 0.0f || u > 1.0f) continue;

                    float qx = sy * e1z - sz * e1y;
                    float qy = sz * e1x - sx * e1z;
                    float qz = sx * e1y - sy * e1x;

                    float v = f * (lDir.x * qx + lDir.y * qy + lDir.z * qz);

                    if (v < 0.0f || u + v > 1.0f) continue;

                    float t = f * (e2x * qx + e2y * qy + e2z * qz);

                    if (t > 0 && t < lDist) {
                        shadow = true;
                        break;
                    }
                }

                if (shadow) break;
            }

            if (shadow) continue;

            float NdotL = nrml * -lDir;
            NdotL = NdotL < 0 ? 0 : NdotL;
            Flt3 diff = alb * NdotL;

            finalColr += lMat.Ems & diff;
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
