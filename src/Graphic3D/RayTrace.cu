#include <RayTrace.cuh>

__global__ void clearFrameBuffer(Vec3f *frmbuffer, int frmW, int frmH) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < frmW * frmH) frmbuffer[i] = Vec3f(0, 0, 0);
}

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
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= frmW * frmH) return;

    int x = i % frmW;
    int y = i / frmW;

    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const double EPSILON_1 = 0.001;
    const double EPSILON_2 = 0.00001;
    const int MAX_RAYS = 10;
    const int MAX_DEPTH = 64;

    // Very important note:
    // If mfv.z = -2, the face is a Sphere!
    // The mv[mfv.x] is the center of the sphere
    // The mv[mfv.y].x is the radius of the sphere

    // Iterative ray tracing

    Ray rays[MAX_RAYS] = { primaryRay };
    RayHit hits[MAX_RAYS] = { RayHit() };
    float weights[MAX_RAYS] = { 1.0f };
    Vec3f colr[MAX_RAYS];
    Vec3f vrtx[MAX_RAYS];
    Vec2f txtr[MAX_RAYS];
    Vec3f nrml[MAX_RAYS];

    int stack[MAX_DEPTH];
    int sptr = 0;
    stack[sptr++] = 0; // Start with root

    int rnum = 0;

    for (int r = 0; r < rnum + 1; r++) {
        if (rnum > MAX_RAYS - 4) break;

        Ray &ray = rays[r];
        RayHit &hit = hits[r];

        sptr = 0;
        stack[sptr++] = 0; // Start with root

        while (sptr > 0) {
            int idx = stack[--sptr];
            const DevNode &node = nodes[idx];

            bool hitAABB = node.hitAABB(ray.origin, ray.direction);
            if (!hitAABB) continue;

            if (!node.leaf) {
                stack[sptr++] = node.l;
                stack[sptr++] = node.r;
                continue;
            }

            for (int i = node.l; i < node.r; i++) {
                int fi = fidx[i];
                Vec3i &fv = mfv[fi];

                Vec3f v0 = mv[fv.x];
                Vec3f v1 = mv[fv.y];
                Vec3f v2 = mv[fv.z];

                Vec3f e1 = v1 - v0;
                Vec3f e2 = v2 - v0;
                Vec3f h = ray.direction & e2;
                float a = e1 * h;

                if (a > -EPSILON_2 && a < EPSILON_2) continue;

                float f = 1.0f / a;
                Vec3f s = ray.origin - v0;
                float u = f * (s * h);

                if (u < 0.0f || u > 1.0f) continue;

                Vec3f q = s & e1;
                float v = f * (ray.direction * q);

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
        int fi = hit.idx; int &fm = mfm[fi];
        const Material &mat = mats[fm];
        float hitw = 1 - hit.u - hit.v;

        // Vertex interpolation
        vrtx[r] = ray.origin + ray.direction * hit.t;

        // Normal interpolation
        Vec3i &fn = mfn[fi];
        if (fn.x > -1) {
            Vec3f &n0 = mn[fn.x], &n1 = mn[fn.y], &n2 = mn[fn.z];
            nrml[r] = n0 * hitw + n1 * hit.u + n2 * hit.v;
            nrml[r].norm();
        }

        // Color/Texture interpolation
        if (mat.mapKd > -1) {
            Vec3i &ft = mft[fi];
            Vec2f &t0 = mt[ft.x], &t1 = mt[ft.y], &t2 = mt[ft.z];
            txtr[r] = t0 * hitw + t1 * hit.u + t2 * hit.v;
            // Modulo 1
            txtr[r].x -= floor(txtr[r].x);
            txtr[r].y -= floor(txtr[r].y);

            int mapKd = mat.mapKd;
            int tw = txtrPtr[mapKd].w;
            int th = txtrPtr[mapKd].h;
            int toff = txtrPtr[mapKd].off;

            int txtrX = txtr[r].x * tw;
            int txtrY = txtr[r].y * th;

            int mapKd2 = txtrX + txtrY * tw + toff;
            colr[r] = txtrFlat[mapKd2];
        } else {
            colr[r] = mat.Kd;
        }

        // Light ray
        Vec3f lightDir = vrtx[r] - lightSrc;
        float lightDist = lightDir.mag();
        lightDir.norm();

        Vec3f shadwColor(0, 0, 0);
        float lightIntens = 1.0f;
        int lightPass = 0;

        sptr = 0;
        stack[sptr++] = 0; // Start with root

        while (sptr > 0) {
            int idx = stack[--sptr];
            const DevNode &node = nodes[idx];

            bool hitAABB = node.hitAABB(vrtx[r], lightDir);
            if (!hitAABB) continue;

            if (!node.leaf) {
                stack[sptr++] = node.l;
                stack[sptr++] = node.r;
                continue;
            }

            for (int i = node.l; i < node.r; i++) {
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
                            Vec3i &ft = mft[i];
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
        }

        if (lightPass > 0) shadwColor /= lightPass;

        // Limit light intesnse to 0.3 - 1.0
        lightIntens = 0.3 + lightIntens * 0.7;

        colr[r] = colr[r] * lightIntens + shadwColor * (1 - lightIntens);

        if (mat.Phong) {
            float diff = -lightDir * nrml[r];
            diff = diff < 0 ? 0 : diff;

            diff = 0.3 + diff * 0.7;
            colr[r] *= diff;
        }

        if (mat.reflect > 0.0f) {
            float weightLeft = weights[r] * mat.reflect;
            weights[r] *= (1 - mat.reflect);

            Vec3f n = nrml[r] * ray.direction > 0 ? -nrml[r] : nrml[r];
            Vec3f reflDir = ray.reflect(n);
            Vec3f reflOrigin = vrtx[r] + n * EPSILON_1;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        } else if (mat.reflect == -1) {
            // Schlick's approximation
            float cosI = (-ray.direction) * nrml[r];
            if (cosI < 0) cosI = -cosI;
            // Find the fresnel coefficient
            float R = pow(1 - cosI, 5);

            float weightLeft = weights[r] * R;
            weights[r] *= (1 - R);

            Vec3f n = nrml[r] * ray.direction > 0 ? -nrml[r] : nrml[r];
            Vec3f reflDir = ray.reflect(n);
            Vec3f reflOrigin = vrtx[r] + n * EPSILON_1;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (mat.transmit > 0.0f) {
            float weightLeft = weights[r] * mat.transmit;
            weights[r] *= (1 - mat.transmit);

            Vec3f transOrg = vrtx[r] + ray.direction * EPSILON_1;

            rays[++rnum] = Ray(transOrg, ray.direction);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (mat.Fresnel > 0.0f) {
            float weightLeft = weights[r] * mat.Fresnel;
            weights[r] *= (1 - mat.Fresnel);

            // Schlick's approximation
            float cosI = (-ray.direction) * nrml[r];
            if (cosI < 0) cosI = -cosI;

            // Find the fresnel coefficient
            float R = pow(1 - cosI, 5);
            float Rrefl = R * weightLeft;
            float Rrefr = (1 - R) * weightLeft;

            // Refraction (for the time being just tranparent)
            Vec3f refrDir = ray.direction;
            Vec3f refrOrigin = vrtx[r] + refrDir * EPSILON_1;

            rays[++rnum] = Ray(refrOrigin, refrDir);
            hits[rnum] = RayHit();
            weights[rnum] = Rrefr;

            // Reflection
            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * EPSILON_1;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = Rrefl;
        }
    }

    Vec3f finalColr(0, 0, 0);
    for (int i = 0; i <= rnum; i++) {
        finalColr += colr[i] * weights[i];
    }

    frmbuffer[i] = finalColr;
}