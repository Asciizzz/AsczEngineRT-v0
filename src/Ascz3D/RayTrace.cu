#include <RayTrace.cuh>

#include <curand_kernel.h>

struct RayHit {
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

__device__ Flt4 getTextureColor(
    float u, float v, Flt4 *txtrFlat, TxtrPtr *txtrPtr, int AlbMap
) {
    u -= floor(u);
    v -= floor(v);

    int tw = txtrPtr[AlbMap].w;
    int th = txtrPtr[AlbMap].h;
    int toff = txtrPtr[AlbMap].off;

    int txtrX = (int)(u * tw);
    int txtrY = (int)(v * th);

    int t = txtrX + txtrY * tw + toff;
    return txtrFlat[t];
}

// Ray intersection

__device__ RayHit RayHitTriangle(const Flt3 &o, const Flt3 &d, const Flt3 &v0, const Flt3 &v1, const Flt3 &v2) {
    RayHit hit;

    Flt3 e1 = v1 - v0;
    Flt3 e2 = v2 - v0;
    Flt3 h = d ^ e2;
    float a = e1 * h;

    if (a == 0) return hit;

    float f = 1.0f / a;
    Flt3 s = o - v0;
    float u = f * (s * h);

    if (u < 0.0f || u > 1.0f) return hit;

    Flt3 q = s ^ e1;
    float v = f * (d * q);

    if (v < 0.0f || u + v > 1.0f) return hit;

    float t = f * (e2 * q);

    if (t > 0) {
        hit.idx = 1;
        hit.t = t;
        hit.u = u;
        hit.v = v;
    }

    return hit;
}

__device__ RayHit RayHitSphere(const Flt3 &o, const Flt3 &d, const Flt3 &sc, float sr) {
    RayHit hit;

    Flt3 l = sc - o;
    float tca = l * d;
    float d2 = l * l - tca * tca;

    if (d2 > sr * sr) return hit;

    float thc = sqrt(sr * sr - d2);
    float t0 = tca - thc;
    float t1 = tca + thc;

    t0 = t0 < 0 ? t1 : t0;

    if (t0 > 0) {
        hit.idx = 1;
        hit.t = t0;
    }

    return hit;
}

__device__ RayHit RayHitGeom(const Flt3 &o, const Flt3 &d, AzGeom &g, Flt3 *mv) {
    RayHit hit;

    if (g.type == AzGeom::TRIANGLE) {
        Int3 &fv = g.tri.v;

        Flt3 v0 = mv[fv.x];
        Flt3 v1 = mv[fv.y];
        Flt3 v2 = mv[fv.z];

        hit = RayHitTriangle(o, d, v0, v1, v2);
    }
    else if (g.type == AzGeom::SPHERE) {
        int cIdx = g.sph.c;

        Flt3 sc = mv[cIdx];
        float sr = g.sph.r;

        hit = RayHitSphere(o, d, sc, sr);
    }

    return hit;
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
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdxs, DevNode *nodes, int nNum // BVH data
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
                int gi = gIdxs[i];

                RayHit h = RayHitGeom(ray.o, ray.d, geom[gi], mv);
                if (h.idx == -1) continue;

                if (h.t < hit.t) {
                    hit = h;
                    hit.idx = gi;
                }
            }
        }

        if (hit.idx == -1) continue;

        // Get the face data
        int hIdx = hit.idx;
        const AzGeom &gHit = geom[hIdx];
        const Material &hMat = mats[gHit.m];

        float hitw = 1 - hit.u - hit.v;

        // Interpolated vertex
        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Interpolated normal
        Flt3 nrml;
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1) {
                nrml = mn[tn.x] * hitw + mn[tn.y] * hit.u + mn[tn.z] * hit.v;
            }
        }
        else if (gHit.type == AzGeom::SPHERE) {
            const int &cIdx = geom[hIdx].sph.c;
            nrml = (vrtx - mv[cIdx]).norm();
        }

        Flt3 alb;
        if (hMat.AlbMap > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                Flt2 txtr = mt[tt.x] * hitw + mt[tt.y] * hit.u + mt[tt.z] * hit.v;

                Flt4 txColr = getTextureColor(txtr.x, txtr.y, txtrFlat, txtrPtr, hMat.AlbMap);

                if (txColr.w < 0.98f && rs_top + 1 < MAX_RAYS) {
                    // Create a new ray
                    float wLeft = ray.w * (1 - txColr.w);
                    ray.w *= txColr.w;

                    rstack[rs_top++] = Ray(
                        vrtx + ray.d * EPSILON_1, ray.d, wLeft, ray.Ior
                    );
                    if (ray.w < 0.05f) continue;
                }

                alb = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / M_2_PI;
                float v = phi / M_PI;

                Flt4 txColr = getTextureColor(u, v, txtrFlat, txtrPtr, hMat.AlbMap);
                alb = txColr.f3();
            }
        } else {
            alb = hMat.Alb;
        }

        // Lighting and shading
        float NdotL = nrml * ray.d;
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
            const Material &lMat = mats[lGeom.m];

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
                    int gi = gIdxs[i];
                    if (gi == hIdx || gi == lIdx) continue;

                    RayHit h = RayHitGeom(lPos, lDir, geom[gi], mv);
                    if (h.idx > -1 && h.t < lDist) {
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
        if (hMat.Tr > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * hMat.Tr;
            ray.w *= (1 - hMat.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;

            rstack[rs_top++] = Ray(rO, ray.d, wLeft, hMat.Ior);
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


__global__ void pathtraceKernel(
    AsczCam camera, unsigned int *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdxs, DevNode *nodes, int nNum // BVH data
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
    int maxBounce = 4;
    int rayPerBounce = 256;

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
                int gi = gIdxs[i];
                if (gi == ray.ignore) continue;

                RayHit h = RayHitGeom(ray.o, ray.d, geom[gi], mv);
                if (h.idx == -1) continue;

                if (h.t < hit.t) {
                    hit = h;
                    hit.idx = gi;
                }
            }
        }

        if (hit.idx == -1) continue;

        // Get the face data
        int hIdx = hit.idx;
        const AzGeom &gHit = geom[hIdx];
        const Material &hMat = mats[gHit.m];

        float hitw = 1 - hit.u - hit.v;

        // Interpolated vertex
        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Interpolated normal
        Flt3 nrml;
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1) {
                nrml = mn[tn.x] * hitw + mn[tn.y] * hit.u + mn[tn.z] * hit.v;
            }
        }
        else if (gHit.type == AzGeom::SPHERE) {
            const int &cIdx = geom[hIdx].sph.c;
            nrml = (vrtx - mv[cIdx]).norm();
        }

        Flt3 alb;
        if (hMat.AlbMap > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                Flt2 txtr = mt[tt.x] * hitw + mt[tt.y] * hit.u + mt[tt.z] * hit.v;

                Flt4 txColr = getTextureColor(txtr.x, txtr.y, txtrFlat, txtrPtr, hMat.AlbMap);

                if (txColr.w < 0.98f && rs_top + 1 < MAX_RAYS) {
                    // Create a new ray
                    float wLeft = ray.w * (1 - txColr.w);
                    ray.w *= txColr.w;

                    rstack[rs_top++] = Ray(
                        vrtx + ray.d * EPSILON_1, ray.d, wLeft, ray.Ior
                    );
                    if (ray.w < 0.05f) continue;
                }

                alb = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / M_2_PI;
                float v = phi / M_PI;

                Flt4 txColr = getTextureColor(u, v, txtrFlat, txtrPtr, hMat.AlbMap);
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
            const Material &lMat = mats[lGeom.m];

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
                    int gi = gIdxs[i];
                    if (gi == hIdx || gi == lIdx) continue;

                    RayHit h = RayHitGeom(lPos, lDir, geom[gi], mv);
                    if (h.idx > -1 && h.t < lDist) {
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
                Flt3 rO = vrtx + nrml * EPSILON_1;
                Flt3 rD = randomHemisphereSample(&rnd, nrml);
                float NdotL = nrml * rD;
                float rW = weightPerRay * NdotL;

                Ray rRay = Ray(rO, rD, rW, ray.Ior, hIdx);

                rstack[rs_top++] = rRay;
            }
        }

        // ======== Additional rays ========

        // Transparent
        if (hMat.Tr > 0.0f && rs_top + 1 < MAX_RAYS) {
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

    int r = (int)(resultColr.x * 255);
    int g = (int)(resultColr.y * 255);
    int b = (int)(resultColr.z * 255);

    frmbuffer[tIdx] = (r << 16) | (g << 8) | b;
}