#include <RayTrace.cuh>

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *frmbuffer, int frmW, int frmH, // In-out
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Vec3f *mv, Vec2f *mt, Vec3f *mn, // Primitive data
    Vec3i *mfv, Vec3i *mft, Vec3i *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH data
    int *fidx, DevNode *nodes, int nNum,

    Vec3f lightSrc
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    // Clear the frame buffer
    frmbuffer[tIdx] = Vec3f(0, 0, 0);

    int x = tIdx % frmW;
    int y = tIdx / frmW;

    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const int MAX_RAYS = 10;
    const int MAX_DEPTH = 32;

    // Very important note:
    // If mfv.z = -2, the face is a Sphere!
    // The mv[mfv.x] is the center of the sphere
    // The mv[mfv.y].x is the radius of the sphere

    // Iterative ray tracing

    Ray rstack[MAX_RAYS] = { primaryRay };
    int r_top = 1; // Ray stack top

    int nstack[MAX_DEPTH];
    int ns_top = 0; // Node stack top

    int rnum = 0;
    Vec3f resultColr = Vec3f(0, 0, 0);
    while (r_top > 0) {
        Ray &ray = rstack[--r_top];
        RayHit hit;

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
                int fi = fidx[i];
                Vec3i &fv = mfv[fi];

                Vec3f v0 = mv[fv.x];
                Vec3f v1 = mv[fv.y];
                Vec3f v2 = mv[fv.z];

                Vec3f e1 = v1 - v0;
                Vec3f e2 = v2 - v0;
                Vec3f h = ray.d & e2;
                float a = e1 * h;

                if (a > -EPSILON_2 && a < EPSILON_2) continue;

                float f = 1.0f / a;
                Vec3f s = ray.o - v0;
                float u = f * (s * h);

                if (u < 0.0f || u > 1.0f) continue;

                Vec3f q = s & e1;
                float v = f * (ray.d * q);

                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * (e2 * q);

                if (t > EPSILON_2 && t < hit.t) {
                    hit.hit = true;
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

        if (!hit.hit) continue;



        // Get the face data
        int hidx = hit.idx; int &fm = mfm[hidx];
        float hitw = 1 - hit.u - hit.v;
        const Material &mat = mats[fm];

        // Vertex interpolation
        Vec3f vrtx = ray.o + ray.d * hit.t;

        // Normal interpolation
        Vec3i &fn = mfn[hidx];
        Vec3f nrml;
        if (fn.x > -1) {
            Vec3f &n0 = mn[fn.x], &n1 = mn[fn.y], &n2 = mn[fn.z];
            nrml = n0 * hitw + n1 * hit.u + n2 * hit.v;
            nrml.norm();
        }

        Vec3f colr;
        // Color/Texture interpolation
        if (mat.mapKd > -1) {
            Vec3i &ft = mft[hidx];
            Vec2f &t0 = mt[ft.x], &t1 = mt[ft.y], &t2 = mt[ft.z];
            Vec2f txtr = t0 * hitw + t1 * hit.u + t2 * hit.v;
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
            colr = txtrFlat[mapKd2];
        } else {
            colr = mat.Kd;
        }

        // Light ray
        Vec3f lightDir = vrtx - lightSrc;
        float lightDist = lightDir.mag();
        lightDir /= lightDist;
        Vec3f lightDirInv = 1.0f / lightDir;

        Vec3f shadwColor(0, 0, 0);
        float lightIntens = 1.0f;
        int lightPass = 0;

        ns_top = 0;
        nstack[ns_top++] = 0; // Start with root

        while (ns_top > 0) {
            int idx = nstack[--ns_top];
            DevNode &node = nodes[idx];

            float hitDist = node.hitDist(lightSrc, lightDirInv);
            if (hitDist < 0 || hitDist > lightDist) continue;

            if (!node.leaf) {
                float ldist = nodes[node.l].hitDist(lightSrc, lightDirInv);
                float rdist = nodes[node.r].hitDist(lightSrc, lightDirInv);

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
                int fi = fidx[i];
                if (fi == hit.idx) continue;

                Vec3i &fv = mfv[fi];

                Vec3f v0 = mv[fv.x];
                Vec3f v1 = mv[fv.y];
                Vec3f v2 = mv[fv.z];

                Vec3f e1 = v1 - v0;
                Vec3f e2 = v2 - v0;
                Vec3f h = lightDir & e2;
                float a = e1 * h;

                if (a > -EPSILON_2 && a < EPSILON_2) continue;

                float f = 1.0f / a;
                Vec3f s = lightSrc - v0;
                float u = f * (s * h);

                if (u < 0.0f || u > 1.0f) continue;

                Vec3f q = s & e1;
                float v = f * (lightDir * q);

                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * (e2 * q);

                if (t > EPSILON_2 && t < lightDist) {
                    const Material &mat = mats[mfm[fi]];

                    if (mat.transmit > 0.0f) {
                        lightPass++;
                        lightIntens *= mat.transmit;

                        // Perform interpolation to get the color
                        if (mat.mapKd > -1) {
                            Vec3i &ft = mft[fi];
                            float w = 1 - u - v;

                            Vec2f &t0 = mt[ft.x], &t1 = mt[ft.y], &t2 = mt[ft.z];
                            Vec2f txtr = t0 * w + t1 * u + t2 * v;
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

                            shadwColor += txtrFlat[mapKd2] * mat.transmit;
                        } else {
                            shadwColor += mat.Kd * mat.transmit;
                        }
                    } else {
                        lightPass = 0;
                        lightIntens = 0.0f;
                        shadwColor = Vec3f(0, 0, 0);
                        break;
                    }
                }
            }

            if (lightIntens < 0.01f) break;
        }

        if (lightPass > 0) shadwColor /= lightPass;

        if (mat.Phong) {
            float diff = -lightDir * nrml;
            diff = diff < 0 ? 0 : diff;
            diff = 0.3 + diff * 0.7;

            Vec3f refl = lightDir - nrml * 2 * (lightDir * nrml);
            float spec = lightIntens * pow(refl * ray.d, mat.Ns);
            spec = spec < 0 ? -spec : spec;

            colr *= diff + spec;
        }
        
        // Limit light intensity to 0.3 - 1.0
        lightIntens = 0.3 + lightIntens * 0.7;

        colr = colr * lightIntens + shadwColor * (1 - lightIntens);

        // Reflective
        if (mat.reflect > 0.0f && r_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mat.reflect;
            ray.w *= (1 - mat.reflect);

            Vec3f reflDir = ray.reflect(nrml);
            Vec3f reflOrigin = vrtx + nrml * EPSILON_1;

            rstack[r_top++] = Ray(reflOrigin, reflDir, ray.Ni);
            rstack[r_top - 1].w = wLeft;
        }
        // Transparent
        else if (mat.transmit > 0.0f && r_top + 1 < MAX_RAYS) {
            float wLeft = ray.w * mat.transmit;
            ray.w *= (1 - mat.transmit);

            Vec3f transOrg = vrtx + ray.d * EPSILON_1;

            rstack[r_top++] = Ray(transOrg, ray.d, mat.Ni);
            rstack[r_top - 1].w = wLeft;
        }
        // Fresnel effect
        else if (mat.Fresnel > 0.0f && r_top + 2 < MAX_RAYS) {
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
            Vec3f refrDir = ray.d;
            Vec3f refrOrigin = vrtx + refrDir * EPSILON_1;
            rstack[r_top++] = Ray(refrOrigin, refrDir, ray.Ni);
            rstack[r_top - 1].w = Rrefr;

            // Reflection
            Vec3f reflDir = ray.reflect(nrml);
            Vec3f reflOrigin = vrtx + nrml * EPSILON_1;
            rstack[r_top++] = Ray(reflOrigin, reflDir, ray.Ni);
            rstack[r_top - 1].w = Rrefl;
        }

        resultColr += colr * ray.w;
    }

    frmbuffer[tIdx] = resultColr;
}