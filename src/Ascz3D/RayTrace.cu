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

__device__ RayHit RayHitGeom(const Flt3 &o, const Flt3 &d, AzGeom &g, float *vx, float *vy, float *vz) {
    RayHit hit;

    if (g.type == AzGeom::TRIANGLE) {
        Int3 &fv = g.tri.v;

        Flt3 v0 = { vx[fv.x], vy[fv.x], vz[fv.x] };
        Flt3 v1 = { vx[fv.y], vy[fv.y], vz[fv.y] };
        Flt3 v2 = { vx[fv.z], vy[fv.z], vz[fv.z] };

        hit = RayHitTriangle(o, d, v0, v1, v2);
    }
    else if (g.type == AzGeom::SPHERE) {
        int cIdx = g.sph.c;

        Flt3 sc = { vx[cIdx], vy[cIdx], vz[cIdx] };
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
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Materials
    AzMtl *mats,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // Geometry data
    AzGeom *geom, int gNum,
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

                RayHit h = RayHitGeom(ray.o, ray.d, geom[gi], vx, vy, vz);
                if (h.idx == -1) continue;

                if (h.t < hit.t) {
                    hit = h;
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
            if (tn.x > -1) {
                nrml.x = nx[tn.x] * hitw + nx[tn.y] * hit.u + nx[tn.z] * hit.v;
                nrml.y = ny[tn.x] * hitw + ny[tn.y] * hit.u + ny[tn.z] * hit.v;
                nrml.z = nz[tn.x] * hitw + nz[tn.y] * hit.u + nz[tn.z] * hit.v;
            }
        }
        else if (gHit.type == AzGeom::SPHERE) {
            int cIdx = geom[hIdx].sph.c;
            nrml.x = vrtx.x - vx[cIdx];
            nrml.y = vrtx.y - vy[cIdx];
            nrml.z = vrtx.z - vz[cIdx];
        }

        Flt3 alb;
        if (hMat.AlbMap > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                float u = tx[tt.x] * hitw + tx[tt.y] * hit.u + tx[tt.z] * hit.v;
                float v = ty[tt.x] * hitw + ty[tt.y] * hit.u + ty[tt.z] * hit.v;

                Flt4 txColr = getTextureColor(u, v, tr, tg, tb, ta, tw, th, toff, hMat.AlbMap);
                alb = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / M_2_PI;
                float v = phi / M_PI;

                Flt4 txColr = getTextureColor(u, v, tr, tg, tb, ta, tw, th, toff, hMat.AlbMap);
                alb = txColr.f3();
            }
        } else {
            alb = hMat.Alb;
        }

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
                lPos.x = (vx[tv.x] + vx[tv.y] + vx[tv.z]) / 3;
                lPos.y = (vy[tv.x] + vy[tv.y] + vy[tv.z]) / 3;
                lPos.z = (vz[tv.x] + vz[tv.y] + vz[tv.z]) / 3;
            }
            else if (lGeom.type == AzGeom::SPHERE) {
                lPos.x = vx[lGeom.sph.c];
                lPos.y = vy[lGeom.sph.c];
                lPos.z = vz[lGeom.sph.c];
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

                    RayHit h = RayHitGeom(lPos, lDir, geom[gi], vx, vy, vz);
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

    int r = (int)(resultColr.x * 255);
    int g = (int)(resultColr.y * 255);
    int b = (int)(resultColr.z * 255);

    frmbuffer[tIdx] = (r << 16) | (g << 8) | b;
}
