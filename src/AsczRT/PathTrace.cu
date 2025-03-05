#include <PathTrace.cuh>
#include <AzDevMath.cuh>

#include <curand_kernel.h>

__device__ Flt3 randomHemisphereSample(curandState *rnd, const Flt3 &n, float exponent = 2.0f) {
    float r1 = curand_uniform(rnd);  
    float r2 = curand_uniform(rnd);

    // Control distribution sharpness with an exponent
    float theta = acos(pow(1.0f - r1, 1.0f / (exponent + 1.0f)));  // Sharper bias to normal
    float phi = 2.0f * M_PI * r2;  

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



__global__ void pathtraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats, int *lSrc, int lNum, 
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    int randSeed
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    curandState rnd;
    curand_init(randSeed + tIdx, tIdx, 0, &rnd);

    int tX = tIdx % frmW, tY = tIdx / frmW;

    Ray ray = camera.castRay(tX, tY, frmW, frmH, curand_uniform(&rnd), curand_uniform(&rnd));

    const int MAX_NODES = 64;
    const int MAX_BOUNCES = 4;

    int nstack[MAX_NODES];
    int ns_top = 0;

    Flt3 throughput = 1.0f; // Energy multiplier
    Flt3 radiance = 0.0f; // Accumulated radiance

    for (int b = 0; b < MAX_BOUNCES; ++b) {
        int hidx = -1;
        float ht = 1e9f;
        float hu = 0.0f;
        float hv = 0.0f;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (mi_x[nidx] - ray.o.x) * ray.invd.x;
            float t2n = (mx_x[nidx] - ray.o.x) * ray.invd.x;
            float t3n = (mi_y[nidx] - ray.o.y) * ray.invd.y;
            float t4n = (mx_y[nidx] - ray.o.y) * ray.invd.y;
            float t5n = (mi_z[nidx] - ray.o.z) * ray.invd.z;
            float t6n = (mx_z[nidx] - ray.o.z) * ray.invd.z;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = ray.o.x < mi_x[nidx] | ray.o.x > mx_x[nidx] |
                        ray.o.y < mi_y[nidx] | ray.o.y > mx_y[nidx] |
                        ray.o.z < mi_z[nidx] | ray.o.z > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > ht) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - ray.o.x) * ray.invd.x;
                float t2l = (mx_x[tcl] - ray.o.x) * ray.invd.x;
                float t3l = (mi_y[tcl] - ray.o.y) * ray.invd.y;
                float t4l = (mx_y[tcl] - ray.o.y) * ray.invd.y;
                float t5l = (mi_z[tcl] - ray.o.z) * ray.invd.z;
                float t6l = (mx_z[tcl] - ray.o.z) * ray.invd.z;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = ray.o.x < mi_x[tcl] | ray.o.x > mx_x[tcl] |
                            ray.o.y < mi_y[tcl] | ray.o.y > mx_y[tcl] |
                            ray.o.z < mi_z[tcl] | ray.o.z > mx_z[tcl];
                float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - ray.o.x) * ray.invd.x;
                float t2r = (mx_x[tcr] - ray.o.x) * ray.invd.x;
                float t3r = (mi_y[tcr] - ray.o.y) * ray.invd.y;
                float t4r = (mx_y[tcr] - ray.o.y) * ray.invd.y;
                float t5r = (mi_z[tcr] - ray.o.z) * ray.invd.z;
                float t6r = (mx_z[tcr] - ray.o.z) * ray.invd.z;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = ray.o.x < mi_x[tcr] | ray.o.x > mx_x[tcr] |
                            ray.o.y < mi_y[tcr] | ray.o.y > mx_y[tcr] |
                            ray.o.z < mi_z[tcr] | ray.o.z > mx_z[tcr];
                float rdist = ((tmaxr < tminr | tminr < 0) ? -1 : tminr) * rOut;


                // Child ordering for closer intersection and early exit
                bool lcloser = ldist < rdist;

                nstack[ns_top] = tcr * lcloser + tcl * !lcloser;
                ns_top += (rdist >= 0) * lcloser + (ldist >= 0) * !lcloser;

                nstack[ns_top] = tcl * lcloser + tcr * !lcloser;
                ns_top += (ldist >= 0) * lcloser + (rdist >= 0) * !lcloser;

                continue;
            }

            for (int i = pl[nidx]; i < pr[nidx]; ++i) {
                int gi = gIdx[i];

                bool hit = gi != ray.ignore;

                float e1x = vx[fv1[gi]] - vx[fv0[gi]];
                float e1y = vy[fv1[gi]] - vy[fv0[gi]];
                float e1z = vz[fv1[gi]] - vz[fv0[gi]];

                float e2x = vx[fv2[gi]] - vx[fv0[gi]];
                float e2y = vy[fv2[gi]] - vy[fv0[gi]];
                float e2z = vz[fv2[gi]] - vz[fv0[gi]];

                float hx = ray.d.y * e2z - ray.d.z * e2y;
                float hy = ray.d.z * e2x - ray.d.x * e2z;
                float hz = ray.d.x * e2y - ray.d.y * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = a == 0.0f ? 1.0f : a;

                float f = 1.0f / a;

                float sx = ray.o.x - vx[fv0[gi]];
                float sy = ray.o.y - vy[fv0[gi]];
                float sz = ray.o.z - vz[fv0[gi]];

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (ray.d.x * qx + ray.d.y * qy + ray.d.z * qz);

                hit &= v >= 0.0f & u + v <= 1.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < ht;

                ht = t * hit + ht * !hit;
                hu = u * hit + hu * !hit;
                hv = v * hit + hv * !hit;
                hidx = gi * hit + hidx * !hit;
            }
        }

        if (hidx == -1 || hidx == ray.ignore) break;

        // Get the face data
        const AzMtl &hm = mats[fm[hidx]];
        float hw = 1.0f - hu - hv;

        // Vertex interpolation
        Flt3 vrtx = ray.o + ray.d * ht;

        // Texture interpolation (if available)
        bool hasTxtr = hm.AlbMap > -1;
        int t0 = ft0[hidx], t1 = ft1[hidx], t2 = ft2[hidx];
        float t_u = hasTxtr ? tx[t0] * hw + tx[t1] * hu + tx[t2] * hv : 0.0f;
        float t_v = hasTxtr ? ty[t0] * hw + ty[t1] * hu + ty[t2] * hv : 0.0f;
        t_u -= floor(t_u); t_v -= floor(t_v);

        int t_w   = hasTxtr ? tw[hm.AlbMap] : 0;
        int t_h   = hasTxtr ? th[hm.AlbMap] : 0;
        int t_off = hasTxtr ? toff[hm.AlbMap] : 0;

        int t_x = (int)(t_u * t_w);
        int t_y = (int)(t_v * t_h);
        int t_idx = t_off + t_y * t_w + t_x;

        Flt3 alb = hasTxtr ? Flt3(tr[t_idx], tg[t_idx], tb[t_idx]) : hm.Alb;

        // Normal interpolation
        Flt3 nrml; bool hasNrml = fn0[hidx] > -1;
        int n0 = fn0[hidx], n1 = fn1[hidx], n2 = fn2[hidx];
        nrml.x = hasNrml ? nx[n0] * hw + nx[n1] * hu + nx[n2] * hv : 0.0f;
        nrml.y = hasNrml ? ny[n0] * hw + ny[n1] * hu + ny[n2] * hv : 0.0f;
        nrml.z = hasNrml ? nz[n0] * hw + nz[n1] * hu + nz[n2] * hv : 0.0f;

        radiance += throughput & hm.Ems;

        throughput &= alb;

        // Indirect lighting
        ray.o = vrtx;
        ray.d = hm.Rf ? ray.d - nrml * 2.0f * (nrml * ray.d) :
                hm.Tr ? ray.d : randomHemisphereSample(&rnd, nrml);
        ray.invd = 1.0f / ray.d;
        ray.ignore = hidx;
    }

    frmbuffer[tIdx] = radiance;
}
