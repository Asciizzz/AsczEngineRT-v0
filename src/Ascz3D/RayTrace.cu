#include <RayTrace.cuh>

#include <curand_kernel.h>

struct RayHit {
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

__device__ Flt4 getTextureColor(
    float u, float v, Flt4 *flat, TxPtr *ptr, int AlbMap
) {
    u -= floor(u);
    v -= floor(v);

    int tw = ptr[AlbMap].w;
    int th = ptr[AlbMap].h;
    int toff = ptr[AlbMap].off;

    int tx = (int)(u * tw);
    int ty = (int)(v * th);

    int t = tx + ty * tw + toff;
    return flat[t];
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

    return Flt3(u, v, t);
}

__device__ Flt3 RayHitSphere(const Flt3 &o, const Flt3 &d, const Flt3 &sc, float sr) {
    Flt3 l = sc - o;
    float tca = l * d;
    float d2 = l * l - tca * tca;

    if (d2 > sr * sr) return -1.0f;

    float thc = sqrt(sr * sr - d2);
    float t0 = tca - thc;
    float t1 = tca + thc;

    t0 = t0 < 0 ? t1 : t0;

    return t0 > 0 ? t0 : -1.0f;
}

__device__ Flt3 RayHitGeom(const Flt3 &o, const Flt3 &d, AzGeom &g, Flt3 *mv) {
    if (g.type == AzGeom::TRIANGLE) {
        Int3 &fv = g.tri.v;
        return RayHitTriangle(o, d, mv[fv.x], mv[fv.y], mv[fv.z]);
    }
    else if (g.type == AzGeom::SPHERE) {
        int cIdx = g.sph.c;
        return RayHitSphere(o, d, mv[cIdx], g.sph.r);   
    }
    return -1.0f;
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
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdx, DevNode *nodes, int nNum, // BVH data

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

                Flt3 h = RayHitGeom(ray.o, ray.d, geom[gi], mv);
                if (h.z > 0 && h.z < hit.t) {
                    hit.u = h.x;
                    hit.v = h.y;
                    hit.t = h.z;
                    hit.idx = gi;
                }
            }
        }

        int hIdx = hit.idx;
        if (hIdx == -1) continue;

        // Get the face data
        const AzGeom &gHit = geom[hIdx];
        const AzMtl &hMat = mats[gHit.m];

        float hitw = 1 - hit.u - hit.v;

        // Interpolated vertex
        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Interpolated normal
        Flt3 nrml;
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1)
                nrml = mn[tn.x] * hitw + mn[tn.y] * hit.u + mn[tn.z] * hit.v;
        }
        else if (gHit.type == AzGeom::SPHERE) {
            int cIdx = geom[hIdx].sph.c;
            nrml = (vrtx - mv[cIdx]).norm();
        }

        Flt3 alb;
        if (hMat.AlbMap > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                float u = mt[tt.x].x * hitw + mt[tt.y].x * hit.u + mt[tt.z].x * hit.v;
                float v = mt[tt.x].y * hitw + mt[tt.y].y * hit.u + mt[tt.z].y * hit.v;

                Flt4 txColr = getTextureColor(u, v, tflat, tptr, hMat.AlbMap);
                alb = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / M_2_PI;
                float v = phi / M_PI;

                Flt4 txColr = getTextureColor(u, v, tflat, tptr, hMat.AlbMap);
                alb = txColr.f3();
            }
        } else {
            alb = hMat.Alb;
        }

        // resultColr += alb; continue;

        // Lighting and shading
        float NdotL = falseAmbient ? nrml * ray.d : 0.0f;
        Flt3 finalColr = alb * 0.02f * NdotL * NdotL;

        if (!hMat.Ems.isZero()) {
            resultColr += hMat.Ems * ray.w;
            continue;
        }

        // Direct lighting
        for (int l = 0; l < lNum; ++l) {
            // Get material and geometry data of light
            int lIdx = lSrc[l];
            const AzGeom &lGeom = geom[lIdx];
            const AzMtl &lMat = mats[lGeom.m];

            // Get position based on the geometry type
            Flt3 lPos;
            if (lGeom.type == AzGeom::TRIANGLE) {
                Int3 tv = lGeom.tri.v;
                lPos = (mv[tv.x] + mv[tv.y] + mv[tv.z]) / 3;
            }
            else if (lGeom.type == AzGeom::SPHERE) {
                lPos = mv[lGeom.sph.c];
            }

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

                    Flt3 h = RayHitGeom(lPos, lDir, geom[gi], mv);
                    if (h.z > 0 && h.z < lDist) {
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
        if (hMat.Tr > 0.0f && rs_top + 2 < MAX_RAYS) {
            float wLeft = ray.w * hMat.Tr;
            ray.w *= (1 - hMat.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;
            rstack[rs_top++] = Ray(rO, ray.d, wLeft, hMat.Ior, hIdx);
        }

        resultColr += finalColr * ray.w;
    }

    // Tone mapping
    resultColr = ASESFilm(resultColr);

    float _gamma = 1.0f / 2.2f;
    resultColr = resultColr.pow(_gamma);

    frmbuffer[tIdx] = resultColr;
}


__global__ void pathtraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdx, DevNode *nodes, int nNum // BVH data
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 1024;
    const int MAX_NODES = 64;

    Ray rstack[MAX_RAYS] = { primaryRay };
    int rs_top = 1;

    int nstack[MAX_NODES];
    int ns_top = 0;

    curandState rnd;
    int bounce = 0;
    int maxBounce = 2;
    int rayPerBounce = 512;

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

                Flt3 h = RayHitGeom(ray.o, ray.d, geom[gi], mv);
                if (h.z > 0 && h.z < hit.t) {
                    hit.u = h.x;
                    hit.v = h.y;
                    hit.t = h.z;
                    hit.idx = gi;
                }
            }
        }

        int hIdx = hit.idx;
        if (hIdx == -1) continue;

        // Get the face data
        const AzGeom &gHit = geom[hIdx];
        const AzMtl &hMat = mats[gHit.m];

        float hitw = 1 - hit.u - hit.v;

        // Interpolated vertex
        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Interpolated normal
        Flt3 nrml;
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1)
                nrml = mn[tn.x] * hitw + mn[tn.y] * hit.u + mn[tn.z] * hit.v;
        }
        else if (gHit.type == AzGeom::SPHERE) {
            int cIdx = geom[hIdx].sph.c;
            nrml = (vrtx - mv[cIdx]).norm();
        }

        Flt3 alb;
        if (hMat.AlbMap > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                Flt2 txtr = mt[tt.x] * hitw + mt[tt.y] * hit.u + mt[tt.z] * hit.v;

                Flt4 txColr = getTextureColor(txtr.x, txtr.y, tflat, tptr, hMat.AlbMap);
                alb = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / M_2_PI;
                float v = phi / M_PI;

                Flt4 txColr = getTextureColor(u, v, tflat, tptr, hMat.AlbMap);
                alb = txColr.f3();
            }
        } else {
            alb = hMat.Alb;
        }

        // Lighting and shading
        Flt3 finalColr;

        if (!hMat.Ems.isZero()) {
            resultColr += hMat.Ems * ray.w;
            continue;
        }

        // Direct lighting
        for (int l = 0; l < lNum; ++l) {
            // Get material and geometry data of light
            int lIdx = lSrc[l];
            const AzGeom &lGeom = geom[lIdx];
            const AzMtl &lMat = mats[lGeom.m];

            // Get position based on the geometry type
            Flt3 lPos;
            if (lGeom.type == AzGeom::TRIANGLE) {
                Int3 tv = lGeom.tri.v;
                lPos = (mv[tv.x] + mv[tv.y] + mv[tv.z]) / 3;
            }
            else if (lGeom.type == AzGeom::SPHERE) {
                lPos = mv[lGeom.sph.c];
            }

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

                    Flt3 h = RayHitGeom(lPos, lDir, geom[gi], mv);
                    if (h.z > 0 && h.z < lDist) {
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

        // Indirect lighting
        if (bounce < maxBounce) {
            ++bounce;
            int curRayPerBounce = rayPerBounce / bounce;
            float weightPerRay = ray.w / curRayPerBounce;
            for (int i = 0; i < curRayPerBounce; ++i) {
                Flt3 rD = randomHemisphereSample(&rnd, nrml);
                float NdotL = nrml * rD;
                float rW = weightPerRay * NdotL;

                Ray rRay = Ray(vrtx, rD, rW, ray.Ior, hIdx);

                rstack[rs_top++] = rRay;
            }
        }

        // ======== Additional rays ========

        // Transparent
        if (hMat.Tr > 0.0f && rs_top + 2 < MAX_RAYS) {
            float wLeft = ray.w * hMat.Tr;
            ray.w *= (1 - hMat.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;
            rstack[rs_top++] = Ray(rO, ray.d, wLeft, hMat.Ior, hIdx);
        }

        resultColr += finalColr * ray.w;
    }

    // Tone mapping
    resultColr = ASESFilm(resultColr);

    float _gamma = 1.0f / 2.2f;
    resultColr = resultColr.pow(_gamma);

    frmbuffer[tIdx] = resultColr;
}


__global__ void raycastKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *gIdx, DevNode *nodes, int nNum // BVH data
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray ray = camera.castRay(x, y, frmW, frmH);

    const int MAX_NODES = 64;

    int nstack[MAX_NODES] = { 0 };
    int ns_top = 1;

    int hIdx = -1;
    float u, v, t = 1e9;

    while (ns_top > 0) {
        int nidx = nstack[--ns_top];
        DevNode &node = nodes[nidx];

        float hitDist = node.hitDist(ray.o, ray.invd);
        if (hitDist < 0 || hitDist > t) continue;

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

            Flt3 h = RayHitGeom(ray.o, ray.d, geom[gi], mv);
            if (h.z > 0 && h.z < t) {
                u = h.x;
                v = h.y;
                t = h.z;
                hIdx = gi;
            }
        }
    }

    if (hIdx == -1) {
        frmbuffer[tIdx] = 0;
        return;
    }

    // Get the face data
    const AzGeom &gHit = geom[hIdx];
    const AzMtl &hMat = mats[gHit.m];

    float w = 1 - u - v;

    Flt3 alb;
    if (hMat.AlbMap > -1) {
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tt = geom[hIdx].tri.t;
            float tu = mt[tt.x].x * w + mt[tt.y].x * u + mt[tt.z].x * v;
            float tv = mt[tt.x].y * w + mt[tt.y].y * u + mt[tt.z].y * v;

            Flt4 txColr = getTextureColor(tu, tv, tflat, tptr, hMat.AlbMap);
            alb = txColr.f3();
        }
        else if (gHit.type == AzGeom::SPHERE) {
            float phi = acosf(-mn[hIdx].y);
            float theta = atan2f(-mn[hIdx].z, -mn[hIdx].x) + M_PI;
            float tu = theta / M_2_PI;
            float tv = phi / M_PI;

            Flt4 txColr = getTextureColor(tu, tv, tflat, tptr, hMat.AlbMap);
            alb = txColr.f3();
        }
    } else {
        alb = hMat.Alb;
    }

    frmbuffer[tIdx] = alb;
}