#include <RayTrace.cuh>

__global__ void clearFrameBuffer(Vec3f *framebuffer, int frmW, int frmH) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < frmW * frmH) framebuffer[i] = Vec3f(0, 0, 0);
}

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *framebuffer, int frmW, int frmH, // In-out
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Vec3f *mv, Vec2f *mt, Vec3f *mn, // Primitive data
    Vec3i *mfv, Vec3i *mft, Vec3i *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH in the near future

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

    // Very important note:
    // If mfv.z = -2, the face is a Sphere!
    // The mv[mfv.x] is the center of the sphere
    // The mv[mfv.y].x is the radius of the sphere

    // Iterative ray tracing

    Ray rays[MAX_RAYS] = { primaryRay };
    RayHit hits[MAX_RAYS] = { RayHit() };
    float weights[MAX_RAYS] = { 1.0f };
    Vec3f vrtx[MAX_RAYS];
    Vec2f txtr[MAX_RAYS];
    Vec3f colr[MAX_RAYS];
    Vec3f nrml[MAX_RAYS];

    int rnum = 0;

    for (int r = 0; r < rnum + 1; r++) {
        if (rnum > MAX_RAYS - 4) break;

        Ray &ray = rays[r];
        RayHit &hit = hits[r];

        for (int i = 0; i < fNum; i++) {
            Vec3i &fv = mfv[i];

            // Get the vertices
            Vec3f v0 = mv[fv.x];
            Vec3f v1 = mv[fv.y];
            Vec3f v2 = mv[fv.z];

            if (fv.z == -2) {
                // Sphere
                Vec3f center = v0;
                float radius = v1.x;

                Vec3f l = center - ray.origin;
                float tca = l * ray.direction;
                float d2 = l * l - tca * tca;

                float rSq = radius * radius;

                if (d2 > rSq) continue;

                float thc = sqrt(rSq - d2);
                float t0 = tca - thc;
                float t1 = tca + thc;

                if (t0 < 0) t0 = t1;

                if (t0 > EPSILON_2 && t0 < hit.t) {
                    hit.hit = true;
                    hit.idx = i;
                    hit.t = t0;
                }

                continue;
            }


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
                hit.idx = i;
                hit.t = t;
                hit.u = u;
                hit.v = v;
            }
        }

    // =========================================================================
    // =========================================================================
    // =========================================================================

        if (!hit.hit) continue;

        // Get the face data
        int fIdx = hit.idx; int &fm = mfm[fIdx];
        const Material &mat = mats[fm];
        float hitw = 1 - hit.u - hit.v;

        // Vertex interpolation
        vrtx[r] = ray.origin + ray.direction * hit.t;

        // Normal interpolation
        Vec3i &fn = mfn[fIdx];
        if (fn.x > -1) {
            Vec3f &n0 = mn[fn.x], &n1 = mn[fn.y], &n2 = mn[fn.z];
            nrml[r] = n0 * hitw + n1 * hit.u + n2 * hit.v;
        }

        // Color/Texture interpolation
        if (mat.mapKd > -1) {
            Vec3i &ft = mft[fIdx];
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
        Vec3f lightDir = lightSrc - vrtx[r]; lightDir.norm();
        Vec3f lightOrigin = vrtx[r] + nrml[r] * EPSILON_1;
        Ray lightRay(lightOrigin, lightDir);

        Vec3f shadwColor(0, 0, 0);
        float lightIntens = 1.0f;
        int lightPass = 0;

        for (int i = 0; i < fNum; i++) {
            if (i == fIdx) continue;

            Vec3i &fv = mfv[i];

            // Get the vertices
            Vec3f v0 = mv[fv.x];
            Vec3f v1 = mv[fv.y];
            Vec3f v2 = mv[fv.z];

            Vec3f e1 = v1 - v0;
            Vec3f e2 = v2 - v0;
            Vec3f h = lightRay.direction & e2;
            float a = e1 * h;

            if (a > -EPSILON_2 && a < EPSILON_2) continue;

            float f = 1.0f / a;
            Vec3f s = lightRay.origin - v0;
            float u = f * (s * h);

            if (u < 0.0f || u > 1.0f) continue;

            Vec3f q = s & e1;
            float v = f * (lightRay.direction * q);

            if (v < 0.0f || u + v > 1.0f) continue;

            float t = f * (e2 * q);

            if (t > EPSILON_2) {
                const Material &mat = mats[mfm[i]];

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
                    lightIntens = 0.0f;
                    shadwColor = Vec3f(0, 0, 0);
                    break;
                }
            }
        }

        if (lightPass > 0) shadwColor /= lightPass;

        // Limit light intesnse to 0.3 - 1.0
        lightIntens = 0.3 + lightIntens * 0.7;

        colr[r] = colr[r] * lightIntens + shadwColor * (1 - lightIntens);

        // Apply very basic lighting with light ray from the top
        if (mat.Phong) {
            float diff = nrml[r] * lightDir;
            diff = diff < 0 ? 0 : diff;

            diff = 0.3 + diff * 0.7;
            colr[r] *= diff;
        }

        if (mat.reflect > 0.0f) {
            float weightLeft = weights[r] * mat.reflect;
            weights[r] *= (1 - mat.reflect);

            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * EPSILON_1;

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

            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * EPSILON_1;

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

    framebuffer[i] = finalColr;
}