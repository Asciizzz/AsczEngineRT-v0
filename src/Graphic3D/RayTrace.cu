#include <RayTrace.cuh>

#include <curand_kernel.h>

_dev_ Flt3 getTextureColor(
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
    return txtrFlat[t].f3();
}

_glb_ void realtimeRayTracing(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mtls, // Materials
    // Mesh data
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data

    // BVH data
    int *gIdx, DevNode *nodes, int nNum,

    // Light data
    LightSrc *lSrc, int lNum,

    curandState *randState
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    curandState localRand = randState[tIdx];

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 32;
    const int MAX_DEPTH = 32;

    Ray rstack[MAX_RAYS]; // Ray stack
    int rs_top = 0; // Ray stack top
    rstack[rs_top++] = primaryRay;

    int nstack[MAX_DEPTH]; // Node stack
    int ns_top = 0; // Node stack top

    int rnum = 0;
    Flt3 resultColr = Flt3(0, 0, 0);
    while (rs_top > 0) {
        // Copy before pop
        Ray ray = rstack[--rs_top];
        RayHit hit;

        // Ray with little contribution
        if (ray.w < 0.01f) continue;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];
            DevNode &node = nodes[nidx];

            float hitDist = node.hitDist(ray.o, ray.invd);
            if (hitDist < 0 || hitDist > hit.t) continue;

            if (!node.leaf) {
                float ldist = nodes[node.l].hitDist(ray.o, ray.invd);
                float rdist = nodes[node.r].hitDist(ray.o, ray.invd);

                // Early exit
                if (ldist < 0 && rdist < 0) continue;
                // Push the valid node
                else if (ldist < 0) nstack[ns_top++] = node.r;
                else if (rdist < 0) nstack[ns_top++] = node.l;
                // Push the closest node first
                else {
                    nstack[ns_top++] = ldist < rdist ? node.r : node.l;
                    nstack[ns_top++] = ldist < rdist ? node.l : node.r;
                }

                continue;
            }

            for (int i = node.l; i < node.r; ++i) {
                int gi = gIdx[i];

                if (geom[gi].type == AzGeom::TRIANGLE) {
                    Int3 &fv = geom[gi].tri.v;

                    Flt3 v0 = mv[fv.x];
                    Flt3 v1 = mv[fv.y];
                    Flt3 v2 = mv[fv.z];

                    Flt3 e1 = v1 - v0;
                    Flt3 e2 = v2 - v0;
                    Flt3 h = ray.d ^ e2;
                    float a = e1 * h;

                    if (a > -EPSILON_2 && a < EPSILON_2) continue;

                    float f = 1.0f / a;
                    Flt3 s = ray.o - v0;
                    float u = f * (s * h);

                    if (u < 0.0f || u > 1.0f) continue;

                    Flt3 q = s ^ e1;
                    float v = f * (ray.d * q);

                    if (v < 0.0f || u + v > 1.0f) continue;

                    float t = f * (e2 * q);

                    if (t > EPSILON_2 && t < hit.t) {
                        hit.idx = gi;
                        hit.t = t;
                        hit.u = u;
                        hit.v = v;
                    }
                }
                else if (geom[gi].type == AzGeom::SPHERE) {
                    Flt3 sc = geom[gi].sph.c;
                    float sr = geom[gi].sph.r;

                    Flt3 l = sc - ray.o;
                    float tca = l * ray.d;
                    float d2 = l * l - tca * tca;

                    if (d2 > sr * sr) continue;
                    
                    float thc = sqrt(sr * sr - d2);
                    float t0 = tca - thc;
                    float t1 = tca + thc;

                    if (t0 < 0) t0 = t1;

                    if (t0 > EPSILON_2 && t0 < hit.t) {
                        hit.idx = gi;
                        hit.t = t0;
                    }
                }
            }
        }

    // =========================================================================
    // =========================================================================
    // =========================================================================

        if (hit.idx == -1) continue;

        // Get the face data
        int hIdx = hit.idx;
        const AzGeom &gHit = geom[hIdx];

        float hitw = 1 - hit.u - hit.v;
        
        // Vertex, normal and Kd data
        Flt3 vrtx = ray.o + ray.d * hit.t;
        Flt3 nrml;
        Flt3 hitKd;

        // Interpolated normal
        if (gHit.type == AzGeom::TRIANGLE) {
            Int3 &tn = geom[hIdx].tri.n;
            if (tn.x > -1) {
                Flt3 &n0 = mn[tn.x], &n1 = mn[tn.y], &n2 = mn[tn.z];
                nrml = (n0 * hitw + n1 * hit.u + n2 * hit.v).norm();
            }
        }
        else if (gHit.type == AzGeom::SPHERE) {
            nrml = (vrtx - gHit.sph.c).norm();
        }

        const Material &mtl = mtls[gHit.m];
        if (mtl.mKd > -1) {
            if (gHit.type == AzGeom::TRIANGLE) {

                Int3 &tt = geom[hIdx].tri.t;
                Flt2 &t0 = mt[tt.x], &t1 = mt[tt.y], &t2 = mt[tt.z];
                Flt2 txtr = t0 * hitw + t1 * hit.u + t2 * hit.v;

                hitKd = getTextureColor(txtr, txtrFlat, txtrPtr, mtl.mKd);
            }
            else if (gHit.type == AzGeom::SPHERE) {
                float phi = acosf(-nrml.y);
                float theta = atan2f(-nrml.z, -nrml.x) + M_PI;
                float u = theta / (2 * M_PI);
                float v = phi / M_PI;

                hitKd = getTextureColor(Flt2(u, v), txtrFlat, txtrPtr, mtl.mKd);
            }
        } else {
            hitKd = mtl.Kd;
        }


        // Light management
        float RdotN = ray.d * nrml;
        RdotN = RdotN < 0 ? -RdotN : RdotN;

        Flt3 finalColr = (mtl.Ka & hitKd) * RdotN;

        for (int l = 0; l < lNum; ++l) {
            const LightSrc &light = lSrc[l];

            Flt3 lPos = light.pos;

            Flt3 lDir = vrtx - lPos;
            float lDist = lDir.mag();
            lDir /= lDist;

            Flt3 lInv = 1.0f / lDir;

            ns_top = 0;
            nstack[ns_top++] = 0; // Start with root

            // Values for transparency
            float intens = light.intens;
            Flt3 passColr = light.colr;

            bool shadow = false;
            while (ns_top > 0) {
                int idx = nstack[--ns_top];
                DevNode &node = nodes[idx];

                float hitDist = node.hitDist(lPos, lInv);
                if (hitDist < 0 || hitDist > lDist) continue;

                if (!node.leaf) {
                    float ldist = nodes[node.l].hitDist(lPos, lInv);
                    float rdist = nodes[node.r].hitDist(lPos, lInv);

                    if (ldist < 0 && rdist < 0) continue;
                    else if (ldist < 0) nstack[ns_top++] = node.r;
                    else if (rdist < 0) nstack[ns_top++] = node.l;
                    else {
                        nstack[ns_top++] = ldist < rdist ? node.r : node.l;
                        nstack[ns_top++] = ldist < rdist ? node.l : node.r;
                    }

                    continue;
                }

                for (int i = node.l; i < node.r; ++i) {
                    int gi = gIdx[i];
                    if (gi == hIdx) continue;
    
                    bool hit = false;
                    float u, v;

                    if (geom[gi].type == AzGeom::TRIANGLE) {
                        Int3 &fv = geom[gi].tri.v;

                        Flt3 v0 = mv[fv.x];
                        Flt3 v1 = mv[fv.y];
                        Flt3 v2 = mv[fv.z];

                        Flt3 e1 = v1 - v0;
                        Flt3 e2 = v2 - v0;
                        Flt3 h = lDir ^ e2;
                        float a = e1 * h;

                        if (a > -EPSILON_2 && a < EPSILON_2) continue;

                        float f = 1.0f / a;
                        Flt3 s = lPos - v0;
                        u = f * (s * h);

                        if (u < 0.0f || u > 1.0f) continue;

                        Flt3 q = s ^ e1;
                        v = f * (lDir * q);

                        if (v < 0.0f || u + v > 1.0f) continue;

                        float t = f * (e2 * q);

                        if (t > EPSILON_2 && t < lDist) {
                            hit = true;
                        }
                    }
                    else if (geom[gi].type == AzGeom::SPHERE) {
                        Flt3 sc = geom[gi].sph.c;
                        float sr = geom[gi].sph.r;

                        Flt3 l = sc - lPos;
                        float tca = l * lDir;
                        float d2 = l * l - tca * tca;

                        if (d2 > sr * sr) continue;

                        float thc = sqrt(sr * sr - d2);
                        float t0 = tca - thc;
                        float t1 = tca + thc;

                        if (t0 < 0) t0 = t1;

                        if (t0 > EPSILON_2 && t0 < lDist) {
                            hit = true;
                        }
                    }

                    if (!hit) continue;

                    const Material &mat2 = mtls[geom[gi].m];

                    if (mat2.Tr < 0.01f) {
                        shadow = true;
                        break;
                    }

                    intens *= mat2.Tr;

                    if (mat2.mKd > -1) {
                        float w = 1 - u - v;

                        Int3 &tt = geom[gi].tri.t;
                        Flt2 &t0 = mt[tt.x], &t1 = mt[tt.y], &t2 = mt[tt.z];
                        Flt2 tx = t0 * w + t1 * u + t2 * v;
                        tx.x -= floor(tx.x);
                        tx.y -= floor(tx.y);

                        passColr += getTextureColor(tx, txtrFlat, txtrPtr, mat2.mKd);
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
            NdotL = NdotL < 0 ? -NdotL : NdotL;
            Flt3 diff = hitKd * NdotL;

            Flt3 refl = Ray::reflect(-lDir, nrml);
            Flt3 spec = mtl.Ks * pow(refl * -ray.d, mtl.Ns);

            finalColr += passColr & (spec + diff) * intens;
        }

        // ======== Additional rays ========

        // Reflective
        if (mtl.reflect > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mtl.reflect;
            ray.w *= (1 - mtl.reflect);

            Flt3 rD = ray.reflect(nrml);
            Flt3 rO = vrtx + nrml * EPSILON_1;

            // For another day
            // Ray perfect = Ray(rO, rD, wLeft, ray.Ni);
            // for (int i = 0; i < 8; i++) {
            //     if (rs_top + 1 >= MAX_RAYS) break;

            //     rstack[rs_top++] = Ray::generateJitteredRay(
            //         perfect, 0.1f, &localRand
            //     );
            //     rstack[rs_top - 1].w = wLeft / 8;
            // }

            rstack[rs_top++] = Ray(rO, rD, wLeft, ray.Ni);
        }
        // Transparent
        else if (mtl.Tr > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mtl.Tr;
            ray.w *= (1 - mtl.Tr);

            Flt3 rO = vrtx + ray.d * EPSILON_1;

            rstack[rs_top++] = Ray(rO, ray.d, wLeft, mtl.Ni);
        }

        resultColr += finalColr * ray.w;
    }

    frmbuffer[tIdx] = resultColr;
}