#include <RayTrace.cuh>

#include <curand_kernel.h>

struct RayHit {
    int idx = -1;
    float t = 1e9;
    float u = 0;
    float v = 0;
};

_dev_ Flt4 getTextureColor(
    Flt2 &txtr, Flt4 *txtrFlat, TxtrPtr *txtrPtr, int mKd
) {
    txtr.x -= floor(txtr.x);
    txtr.y -= floor(txtr.y);

    int tw = txtrPtr[mKd].w;
    int th = txtrPtr[mKd].h;
    int toff = txtrPtr[mKd].off;

    int txtrX = txtr.x * tw;
    int txtrY = txtr.y * th;

    int t = txtrX + txtrY * tw + toff;
    return txtrFlat[t];
}

// Ray intersection

_dev_ RayHit RayHitTriangle(const Flt3 &o, const Flt3 &d, const Flt3 &v0, const Flt3 &v1, const Flt3 &v2) {
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

_dev_ RayHit RayHitSphere(const Flt3 &o, const Flt3 &d, const Flt3 &sc, float sr) {
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

_dev_ RayHit RayHitGeom(const Flt3 &o, const Flt3 &d, AzGeom &g, Flt3 *mv) {
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


_glb_ void raytraceKernel(
    AsczCam camera, unsigned int *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mtls, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *gIdxs, DevNode *nodes, int nNum, // BVH data
    LightSrc *lSrc, int lNum // Light data
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 16;
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

        // Ray with little contribution
        if (ray.w < 0.05f) continue;

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
        const Material &hMtl = mtls[gHit.m];

        float hitw = 1 - hit.u - hit.v;

        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Interpolated normal
        Flt3 nrml;
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1) {
                Flt3 &n0 = mn[tn.x], &n1 = mn[tn.y], &n2 = mn[tn.z];
                nrml = n0 * hitw + n1 * hit.u + n2 * hit.v;
            }
        }
        else if (gHit.type == AzGeom::SPHERE) {
            const int &cIdx = geom[hIdx].sph.c;
            nrml = (vrtx - mv[cIdx]).norm();
        }

        Flt3 hitKd;
        if (hMtl.mKd > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {
                Int3 &tt = geom[hIdx].tri.t;
                Flt2 &t0 = mt[tt.x], &t1 = mt[tt.y], &t2 = mt[tt.z];
                Flt2 txtr = t0 * hitw + t1 * hit.u + t2 * hit.v;

                Flt4 txColr = getTextureColor(txtr, txtrFlat, txtrPtr, hMtl.mKd);

                if (txColr.w < 0.98f && rs_top + 1 < MAX_RAYS) {
                    // Create a new ray
                    float wLeft = ray.w * (1 - txColr.w);
                    ray.w *= txColr.w;

                    rstack[rs_top++] = Ray(
                        vrtx + ray.d * EPSILON_1, ray.d, wLeft, ray.Ni
                    );
                    if (ray.w < 0.05f) continue;
                }

                hitKd = txColr.f3();
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                Flt2 uv = Flt2(theta / M_2_PI, phi / M_PI);

                Flt4 txColr = getTextureColor(uv, txtrFlat, txtrPtr, hMtl.mKd);

                if (txColr.w < 0.98f && rs_top + 1 < MAX_RAYS) {
                    // Create a new ray
                    float wLeft = ray.w * (1 - txColr.w);
                    ray.w *= txColr.w;

                    rstack[rs_top++] = Ray(
                        vrtx + ray.d * EPSILON_1, ray.d, wLeft, ray.Ni
                    );
                    if (ray.w < 0.05f) continue;
                }

                hitKd = txColr.f3();
            }
        } else {
            hitKd = hMtl.Kd;
        }

        // Lighting and shading

        float RdotN = ray.d * nrml;
        RdotN = hMtl.noShade ? 1.0f : RdotN * RdotN;

        Flt3 finalColr = (hMtl.Ka & hitKd) * RdotN;

        // Global illumination

        // Direct lighting
        for (int l = 0; l < lNum; ++l) {
            const LightSrc &light = lSrc[l];

            Flt3 lPos = light.pos;

            Flt3 lDir = vrtx - lPos;
            float lDist = lDir.mag();
            lDir /= lDist;

            Flt3 lInv = 1.0f / lDir;

            ns_top = 0;
            nstack[ns_top++] = 0;

            // Values for transparency
            float intens = light.intens;
            Flt3 passColr = light.colr;

            bool shadow = false;
            while (ns_top > 0) {
                if (hMtl.noShadow) break;

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
                    if (gi == hIdx) continue;

                    const Material &mat2 = mtls[geom[gi].m];
                    if (mat2.noShadow) continue;

                    RayHit h = RayHitGeom(lPos, lDir, geom[gi], mv);
                    if (h.idx == -1 || h.t > lDist) continue;

                    if (mat2.Tr < 0.01f) {
                        shadow = true;
                        break;
                    }

                    intens *= mat2.Tr;

                    if (mat2.mKd > -1) {
                        float hw = 1 - h.u - h.v;

                        Int3 &tt = geom[gi].tri.t;
                        Flt2 &t0 = mt[tt.x], &t1 = mt[tt.y], &t2 = mt[tt.z];
                        Flt2 tx = t0 * hw + t1 * h.u + t2 * h.v;
                        tx.x -= floor(tx.x);
                        tx.y -= floor(tx.y);

                        Flt4 txColr = getTextureColor(tx, txtrFlat, txtrPtr, mat2.mKd);
                        passColr += txColr.f3() * txColr.w;
                    } else {
                        passColr += mat2.Kd;
                    }
                }

                if (shadow) break;
            }

            if (shadow) continue;

            // Exponential falloff
            if (light.falloff) {
                float dist = lDist - light.bias;
                float falloff = 1.0f / (1.0f + pow(dist / light.falloffDist, light.exp));
                intens *= falloff;
            }

            float NdotL = nrml * -lDir;
            Flt3 diff = hitKd * NdotL * NdotL;

            Flt3 refl = Ray::reflect(-lDir, nrml);
            Flt3 spec = hMtl.Ks * pow(refl * -ray.d, hMtl.Ns);

            diff = hMtl.noShade ? hitKd : diff;
            spec = hMtl.noShade ? Flt3(0, 0, 0) : spec;

            finalColr += passColr & (spec + diff) * intens;
        }

        // ======== Additional rays ========

        // Reflective
        if (hMtl.reflect > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * hMtl.reflect;
            ray.w *= (1 - hMtl.reflect);

            Flt3 rD = ray.reflect(nrml);
            Flt3 rO = vrtx + nrml * EPSILON_1;

            rstack[rs_top++] = Ray(rO, rD, wLeft, ray.Ni);
        }
        // Transparent
        else if (hMtl.Tr > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * hMtl.Tr;
            ray.w *= (1 - hMtl.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;

            rstack[rs_top++] = Ray(rO, ray.d, wLeft, hMtl.Ni);
        }

        resultColr += finalColr * ray.w;
    }

    // Clamp the color
    resultColr.x = resultColr.x > 1.0f ? 1.0f : resultColr.x;
    resultColr.y = resultColr.y > 1.0f ? 1.0f : resultColr.y;
    resultColr.z = resultColr.z > 1.0f ? 1.0f : resultColr.z;

    // Gamma correction
    float gamma = 2.2f;
    resultColr.x = powf(resultColr.x, 1.0f / gamma);
    resultColr.y = powf(resultColr.y, 1.0f / gamma);
    resultColr.z = powf(resultColr.z, 1.0f / gamma);

    int r = (int)(resultColr.x * 255);
    int g = (int)(resultColr.y * 255);
    int b = (int)(resultColr.z * 255);

    frmbuffer[tIdx] = (r << 16) | (g << 8) | b;
}