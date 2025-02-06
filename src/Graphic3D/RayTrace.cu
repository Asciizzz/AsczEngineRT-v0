#include <RayTrace.cuh>

#include <curand_kernel.h>

__global__ void iterativeRayTracing(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    Int3 *mfv, Int3 *mft, Int3 *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH data
    int *fIdx, DevNode *nodes, int nNum,

    // Light source data
    LightSrc *lSrc, int lNum,

    curandState *randState
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    curandState localRand = randState[tIdx];

    int x = tIdx % frmW, y = tIdx / frmW;
    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 8;
    const int MAX_DEPTH = 32;

    Ray rstack[MAX_RAYS]; // Ray stack
    int rs_top = 0; // Ray stack top
    rstack[rs_top++] = primaryRay;

    int nstack[MAX_DEPTH]; // Node stack
    int ns_top = 0; // Node stack top

    int rnum = 0;
    Flt3 resultColr = Flt3(0, 0, 0);
    while (rs_top > 0) {
        Ray &ray = rstack[--rs_top];
        RayHit hit;

        // Ray with little contribution
        if (ray.w < 0.01f) continue;

        ns_top = 0;
        nstack[ns_top++] = 0; // Start with root

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
                int fi = fIdx[i];
                Int3 &fv = mfv[fi];

                Flt3 v0 = mv[fv.x];
                Flt3 v1 = mv[fv.y];
                Flt3 v2 = mv[fv.z];

                Flt3 e1 = v1 - v0;
                Flt3 e2 = v2 - v0;
                Flt3 h = ray.d & e2;
                float a = e1 * h;

                if (a > -EPSILON_2 && a < EPSILON_2) continue;

                float f = 1.0f / a;
                Flt3 s = ray.o - v0;
                float u = f * (s * h);

                if (u < 0.0f || u > 1.0f) continue;

                Flt3 q = s & e1;
                float v = f * (ray.d * q);

                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * (e2 * q);

                if (t > EPSILON_2 && t < hit.t) {
                    hit.idx = fi;
                    hit.t = t;
                    hit.u = u;
                    hit.v = v;
                }
            }
        }

    // =========================================================================
    // =========================================================================
    // =========================================================================

        if (hit.idx == -1) continue;

        // Get the face data
        int hidx = hit.idx; int &fm = mfm[hidx];
        float hitw = 1 - hit.u - hit.v;
        const Material &mat = mats[fm];

        // Vertex interpolation
        Flt3 vrtx = ray.o + ray.d * hit.t;

        // Normal interpolation
        Int3 &fn = mfn[hidx];
        Flt3 nrml;
        if (fn.x > -1) {
            Flt3 &n0 = mn[fn.x], &n1 = mn[fn.y], &n2 = mn[fn.z];
            nrml = n0 * hitw + n1 * hit.u + n2 * hit.v;
            nrml.norm();
        }

        Flt3 colr;
        // Color/Texture interpolation
        if (mat.mapKd > -1) {
            Int3 &ft = mft[hidx];
            Flt2 &t0 = mt[ft.x], &t1 = mt[ft.y], &t2 = mt[ft.z];
            Flt2 txtr = t0 * hitw + t1 * hit.u + t2 * hit.v;
            // Modulo 1
            txtr.x -= floor(txtr.x);
            txtr.y -= floor(txtr.y);

            int mapKd = mat.mapKd;
            int tw = txtrPtr[mapKd].w;
            int th = txtrPtr[mapKd].h;
            int toff = txtrPtr[mapKd].off;

            int txtrX = txtr.x * tw;
            int txtrY = txtr.y * th;

            int mapKd2 = txtrX + txtrY * tw + toff;
            Flt4 tColr = txtrFlat[mapKd2];

            colr = Flt3(tColr.x, tColr.y, tColr.z);
        } else {
            colr = mat.Kd;
        }

        // Light management
        Flt3 finalColr = Flt3(0, 0, 0);

        for (int l = 0; l < lNum; ++l) {
            const LightSrc &light = lSrc[l];

            Flt3 lDir = vrtx - light.pos;
            float lDist = lDir.mag();
            lDir.norm();
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

                float hitDist = node.hitDist(light.pos, lInv);
                if (hitDist < 0 || hitDist > lDist) continue;

                if (!node.leaf) {
                    float ldist = nodes[node.l].hitDist(light.pos, lInv);
                    float rdist = nodes[node.r].hitDist(light.pos, lInv);

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
                    int fi = fIdx[i];
                    if (fi == hit.idx) continue;

                    Int3 &fv = mfv[fi];

                    Flt3 v0 = mv[fv.x];
                    Flt3 v1 = mv[fv.y];
                    Flt3 v2 = mv[fv.z];

                    Flt3 e1 = v1 - v0;
                    Flt3 e2 = v2 - v0;
                    Flt3 h = lDir & e2;
                    float a = e1 * h;

                    if (a > -EPSILON_2 && a < EPSILON_2) continue;

                    float f = 1.0f / a;
                    Flt3 s = light.pos - v0;
                    float u = f * (s * h);

                    if (u < 0.0f || u > 1.0f) continue;

                    Flt3 q = s & e1;
                    float v = f * (lDir * q);

                    if (v < 0.0f || u + v > 1.0f) continue;

                    float t = f * (e2 * q);

                    if (t > EPSILON_2 && t < lDist) {
                        const Material &mat2 = mats[mfm[fi]];

                        if (mat2.transmit > 0.0f) {
                            intens *= mat2.transmit;
                            passColr += mat2.Kd;
                        } else {
                            shadow = true;
                            break;
                        }
                    }
                }

                if (shadow) break;
            }

            if (shadow) continue;


            // Closer = brighter
            intens *= 1.0f / (lDist * lDist);

            float diff = nrml * -lDir;
            diff = diff < 0 ? -diff : diff;

            Flt3 refl = Ray::reflect(-lDir, nrml);
            float spec = pow(refl * -ray.d, mat.Ns);

            finalColr += passColr * (diff + spec) * intens;
        }

        finalColr.x = finalColr.x < 0.02f ? 0.02f : finalColr.x;
        finalColr.y = finalColr.y < 0.02f ? 0.02f : finalColr.y;
        finalColr.z = finalColr.z < 0.02f ? 0.02f : finalColr.z;

        colr *= finalColr;

        // Reflective
        if (mat.reflect > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mat.reflect;
            ray.w *= (1 - mat.reflect);

            Flt3 reflDir = ray.reflect(nrml);
            Flt3 reflOrigin = vrtx + nrml * EPSILON_1;

            rstack[rs_top++] = Ray(reflOrigin, reflDir, wLeft, ray.Ni);
        }
        // Transparent
        else if (mat.transmit > 0.0f && rs_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mat.transmit;
            ray.w *= (1 - mat.transmit);

            Flt3 transOrg = vrtx + ray.d * EPSILON_1;

            rstack[rs_top++] = Ray(transOrg, ray.d, wLeft, mat.Ni);
        }
        // Fresnel effect
        else if (mat.Fresnel > 0.0f && rs_top + 2 < MAX_RAYS) {
            float wLeft = ray.w * mat.Fresnel;
            ray.w *= (1 - mat.Fresnel);

            // Schlick's approximation
            float cosI = (-ray.d) * nrml;
            cosI = cosI < 0 ? 0 : cosI;

            // Find the fresnel coefficient
            float R = pow(1 - cosI, 5);
            float Rrefl = R * wLeft;
            float Rrefr = (1 - R) * wLeft;

            // Refraction (for the time being just tranparent)
            Flt3 refrDir = ray.d;
            Flt3 refrOrigin = vrtx + refrDir * EPSILON_1;
            rstack[rs_top++] = Ray(refrOrigin, refrDir, Rrefr, ray.Ni);

            // Reflection
            Flt3 reflDir = ray.reflect(nrml);
            Flt3 reflOrigin = vrtx + nrml * EPSILON_1;
            rstack[rs_top++] = Ray(reflOrigin, reflDir, Rrefl, ray.Ni);
        }

        resultColr += colr * ray.w;
    }

    frmbuffer[tIdx] = resultColr;
}