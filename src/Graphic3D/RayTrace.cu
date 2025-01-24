#include <RayTrace.cuh>

__global__ void clearFrameBuffer(Vec3f *framebuffer, int frmW, int frmH) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < frmW * frmH) framebuffer[i] = Vec3f(0, 0, 0);
}

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *framebuffer, int frmW, int frmH, // In-out
    Geom *geoms, int geomNum, // Will be replaced with BVH
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats // Materials
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= frmW * frmH) return;

    int x = i % frmW;
    int y = i / frmW;

    Ray primaryRay = camera.castRay(x, y, frmW, frmH);

    const double EPSILON = 0.001;
    const int MAX_RAYS = 10;

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

    // =========================================================================
    // ======================= Will be replaced with BVH =======================
    // =========================================================================

        for (int g = 0; g < geomNum; g++) {
            const Geom &geom = geoms[g];

            /*
            "WhY dOn'T yOu WrItE a FuNcTiOn tO dO tHe InTeRsEcTiOn?"

            BECAUSE IT KILLED THE PERFORMANCE, OKAY?
            IDK WHY, BUT IT PLUMMETED THE PERFORMANCE
            FROM 60 FPS TO 20 FPS, SO I'M NOT GONNA DO IT
            */

            switch (geom.type) {
                case Geom::TRIANGLE: {
                    const Triangle &tri = geom.tri;
                    Vec3f e1 = tri.v1 - tri.v0;
                    Vec3f e2 = tri.v2 - tri.v0;
                    Vec3f h = ray.direction & e2;
                    float a = e1 * h;

                    if (a > -0.00001 && a < 0.00001) continue;

                    float f = 1.0f / a;
                    Vec3f s = ray.origin - tri.v0;
                    float u = f * (s * h);

                    if (u < 0.0f || u > 1.0f) continue;

                    Vec3f q = s & e1;
                    float v = f * (ray.direction * q);

                    if (v < 0.0f || u + v > 1.0f) continue;

                    float t = f * (e2 * q);

                    if (t > 0.00001 && t < hit.t) {
                        hit.hit = true;
                        hit.idx = g;
                        hit.t = t;
                        hit.u = u;
                        hit.v = v;
                    }
                    continue;
                }

                case Geom::SPHERE: {
                    const Sphere &sph = geom.sph;

                    Vec3f l = sph.o - ray.origin;
                    float tca = l * ray.direction;
                    float d2 = l * l - tca * tca;

                    if (d2 > sph.r * sph.r) continue;

                    float thc = sqrt(sph.r * sph.r - d2);
                    float t0 = tca - thc;
                    float t1 = tca + thc;

                    if (t0 < 0) t0 = t1;

                    if (t0 > EPSILON && t0 < hit.t) {
                        hit.hit = true;
                        hit.idx = g;
                        hit.t = t0;
                    }
                    continue;
                }

                case Geom::PLANE: {
                    const Plane &pln = geom.pln;

                    float denom = pln.n * ray.direction;
                    if (denom > -EPSILON && denom < EPSILON) continue; // Parallel

                    float t = -(pln.n * ray.origin + pln.d) / denom;

                    if (t < hit.t && t > EPSILON) {
                        hit.hit = true;
                        hit.idx = g;
                        hit.t = t;
                    }
                    continue;
                }
            }
        }

    // =========================================================================
    // =========================================================================
    // =========================================================================

        if (!hit.hit) continue;

        // Interpolate the hit point
        const Geom &geom = geoms[hit.idx];
        const Material &mat = mats[geom.mat];

        vrtx[r] = ray.origin + ray.direction * hit.t;

        switch (geom.type) {
            case Geom::TRIANGLE: {
                const Triangle &tri = geom.tri;
                float w = 1 - hit.u - hit.v;

                nrml[r] = tri.n0 * w + tri.n1 * hit.u + tri.n2 * hit.v;
                nrml[r].norm();

                if (mat.txtrIdx > -1) {
                    txtr[r] = tri.t0 * w + tri.t1 * hit.u + tri.t2 * hit.v;
                    // // Modulo 1
                    // txtr[r].x -= floor(txtr[r].x);
                    // txtr[r].y -= floor(txtr[r].y);

                    int txtrIdx = mat.txtrIdx;
                    int w = txtrPtr[txtrIdx].w;
                    int h = txtrPtr[txtrIdx].h;
                    int off = txtrPtr[txtrIdx].off;

                    int txtrX = txtr[r].x * w;
                    int txtrY = txtr[r].y * h;

                    int txtrIdx2 = txtrX + txtrY * w + off;
                    colr[r] = txtrFlat[txtrIdx2];
                } else {
                    colr[r] = tri.c0 * w + tri.c1 * hit.u + tri.c2 * hit.v;
                }

                break;
            }

            case Geom::SPHERE: {
                const Sphere &sph = geom.sph;
                nrml[r] = (vrtx[r] - sph.o) / sph.r;
                if (sph.invert) nrml[r] = -nrml[r];

                if (mat.txtrIdx > -1) {
                    Vec3f p = (vrtx[r] - sph.o) / sph.r;
                    float phi = atan2(p.z, p.x);
                    float theta = asin(p.y);

                    float u = 1 - (phi + M_PI) / (2 * M_PI);
                    float v = (theta + M_PI_2) / M_PI;

                    int txtrIdx = mat.txtrIdx;
                    int w = txtrPtr[txtrIdx].w;
                    int h = txtrPtr[txtrIdx].h;
                    int off = txtrPtr[txtrIdx].off;

                    int txtrX = u * w;
                    int txtrY = v * h;

                    int txtrIdx2 = txtrX + txtrY * w + off;
                    colr[r] = txtrFlat[txtrIdx2];
                } else {
                    colr[r] = sph.color;
                }

                break;
            }

            case Geom::PLANE: {
                const Plane &pln = geom.pln;
                colr[r] = pln.color;
                nrml[r] = pln.n;
                break;
            }
        }

        // If the geometry is a sky, ignore the rest
        if (mat.isSky) continue;

        // Test light source
        Vec3f lightSrc(10, 10, 10);

        // Shadow ray
        Vec3f lightDir = lightSrc - vrtx[r];
        Vec3f lightOrigin = vrtx[r] + lightDir * EPSILON;
        lightDir.norm();
        Ray shadowRay(lightOrigin, lightDir);
        bool shadow = false;

        for (int g = 0; g < geomNum; g++) {
            const Geom &geom = geoms[g];
            const Material &mat = mats[geom.mat];

            if (mat.isSky) continue;

            /*
            Highly repetitive code, but doing otherwise
            will absolutely kill performance
            */

            // Triangle
            if (geom.type == Geom::TRIANGLE) {
                const Triangle &tri = geom.tri;
                Vec3f e1 = tri.v1 - tri.v0;
                Vec3f e2 = tri.v2 - tri.v0;
                Vec3f h = shadowRay.direction & e2;
                float a = e1 * h;

                if (a > -EPSILON && a < EPSILON) continue;

                float f = 1.0f / a;
                Vec3f s = shadowRay.origin - tri.v0;
                float u = f * (s * h);

                if (u < 0.0f || u > 1.0f) continue;

                Vec3f q = s & e1;
                float v = f * (shadowRay.direction * q);

                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * (e2 * q);

                if (t > EPSILON) {
                    shadow = true;
                    break;
                }
            // Sphere
            } else if (geom.type == Geom::SPHERE) {
                    const Sphere &sph = geom.sph;

                    Vec3f l = sph.o - shadowRay.origin;
                    float tca = l * shadowRay.direction;
                    float d2 = l * l - tca * tca;

                    if (d2 > sph.r * sph.r) continue;

                    float thc = sqrt(sph.r * sph.r - d2);
                    float t0 = tca - thc;
                    float t1 = tca + thc;

                    if (t0 < 0) t0 = t1;

                    if (t0 > EPSILON) {
                        shadow = true;
                        break;
                    }
            // Plane
            } else if (geom.type == Geom::PLANE) {
                const Plane &pln = geom.pln;

                float denom = pln.n * shadowRay.direction;
                if (denom > -EPSILON && denom < EPSILON) continue; // Parallel

                float t = -(pln.n * shadowRay.origin + pln.d) / denom;

                if (t > EPSILON) {
                    shadow = true;
                    break;
                }
            }
        }

        if (shadow) weights[r] *= 0.3;

        // Apply very basic lighting with light ray from the top
        float diff = nrml[r] * lightDir;
        if (diff < 0) diff = 0;

        diff = 0.3 + diff * 0.7;
        colr[r] *= diff;

        if (mat.reflect > 0.0f) {
            float weightLeft = weights[r] * mat.reflect;
            weights[r] *= (1 - mat.reflect);

            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * EPSILON;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (mat.transmit > 0.0f) {
            float weightLeft = weights[r] * mat.transmit;
            weights[r] *= (1 - mat.transmit);

            Vec3f transOrg = vrtx[r] + ray.direction * EPSILON;

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
            Vec3f refrOrigin = vrtx[r] + refrDir * EPSILON;

            rays[++rnum] = Ray(refrOrigin, refrDir);
            hits[rnum] = RayHit();
            weights[rnum] = Rrefr;

            // Reflection
            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * EPSILON;

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