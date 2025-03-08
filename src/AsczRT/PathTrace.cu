#include <PathTrace.cuh>
#include <AzDevMath.cuh>


__global__ void pathtraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    curandState *rnd
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmW * frmH) return;

    const int MAX_BOUNCES = 8;
    const int MAX_NODES = 64;

    int tX = tIdx % frmW, tY = tIdx / frmW;

    float rnd1 = curand_uniform(&rnd[tIdx]);
    float rnd2 = curand_uniform(&rnd[tIdx]);
    float rnd3 = curand_uniform(&rnd[tIdx]);
    float rnd4 = curand_uniform(&rnd[tIdx]);

    Ray ray = camera.castRay(tX, tY, frmW, frmH,
                            rnd1, rnd2, rnd3, rnd4);

    // Ray direction
    float RD_x = ray.d.x, RD_y = ray.d.y, RD_z = ray.d.z;
    // Ray inverse direction
    float RInvd_x = ray.invd.x, RInvd_y = ray.invd.y, RInvd_z = ray.invd.z;
    // Ray origin
    float RO_x = ray.o.x, RO_y = ray.o.y, RO_z = ray.o.z;
    // Ray ignore
    int RIgnore = ray.ignore;
    // Ray index of refraction
    float RIor = ray.Ior;

    int nstack[MAX_NODES];
    int ns_top = 0;

    float thru_x = 1.0f, thru_y = 1.0f, thru_z = 1.0f; // Throughput
    float radi_x = 0.0f, radi_y = 0.0f, radi_z = 0.0f; // Radiance

    for (int b = 0; b < MAX_BOUNCES; ++b) {
        int hidx = -1;
        float ht = 1e9f;
        float hu = 0.0f;
        float hv = 0.0f;
        float hw = 0.0f;

        ns_top = 0;
        nstack[ns_top++] = 0;

        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (mi_x[nidx] - RO_x) * RInvd_x;
            float t2n = (mx_x[nidx] - RO_x) * RInvd_x;
            float t3n = (mi_y[nidx] - RO_y) * RInvd_y;
            float t4n = (mx_y[nidx] - RO_y) * RInvd_y;
            float t5n = (mi_z[nidx] - RO_z) * RInvd_z;
            float t6n = (mx_z[nidx] - RO_z) * RInvd_z;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = RO_x < mi_x[nidx] | RO_x > mx_x[nidx] |
                        RO_y < mi_y[nidx] | RO_y > mx_y[nidx] |
                        RO_z < mi_z[nidx] | RO_z > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > ht) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - RO_x) * RInvd_x;
                float t2l = (mx_x[tcl] - RO_x) * RInvd_x;
                float t3l = (mi_y[tcl] - RO_y) * RInvd_y;
                float t4l = (mx_y[tcl] - RO_y) * RInvd_y;
                float t5l = (mi_z[tcl] - RO_z) * RInvd_z;
                float t6l = (mx_z[tcl] - RO_z) * RInvd_z;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = RO_x < mi_x[tcl] | RO_x > mx_x[tcl] |
                            RO_y < mi_y[tcl] | RO_y > mx_y[tcl] |
                            RO_z < mi_z[tcl] | RO_z > mx_z[tcl];
                float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - RO_x) * RInvd_x;
                float t2r = (mx_x[tcr] - RO_x) * RInvd_x;
                float t3r = (mi_y[tcr] - RO_y) * RInvd_y;
                float t4r = (mx_y[tcr] - RO_y) * RInvd_y;
                float t5r = (mi_z[tcr] - RO_z) * RInvd_z;
                float t6r = (mx_z[tcr] - RO_z) * RInvd_z;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = RO_x < mi_x[tcr] | RO_x > mx_x[tcr] |
                            RO_y < mi_y[tcr] | RO_y > mx_y[tcr] |
                            RO_z < mi_z[tcr] | RO_z > mx_z[tcr];
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

                bool hit = gi != RIgnore;

                float e1x = vx[fv1[gi]] - vx[fv0[gi]];
                float e1y = vy[fv1[gi]] - vy[fv0[gi]];
                float e1z = vz[fv1[gi]] - vz[fv0[gi]];

                float e2x = vx[fv2[gi]] - vx[fv0[gi]];
                float e2y = vy[fv2[gi]] - vy[fv0[gi]];
                float e2z = vz[fv2[gi]] - vz[fv0[gi]];

                float hx = RD_y * e2z - RD_z * e2y;
                float hy = RD_z * e2x - RD_x * e2z;
                float hz = RD_x * e2y - RD_y * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a * hit;

                float sx = RO_x - vx[fv0[gi]];
                float sy = RO_y - vy[fv0[gi]];
                float sz = RO_z - vz[fv0[gi]];

                // Since 1/a is used twice and division is expensive
                // Store it in f = 1/a
                float f = 1.0f / a;

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (RD_x * qx + RD_y * qy + RD_z * qz);
                float w = 1.0f - u - v;

                hit &= v >= 0.0f & w >= 0.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < ht;

                ht = t * hit + ht * !hit;
                hu = u * hit + hu * !hit;
                hv = v * hit + hv * !hit;
                hw = w * hit + hw * !hit;
                hidx = gi * hit + hidx * !hit;
            }
        }

        if (hidx == -1) break;

        // Get the face data
        const AzMtl &hm = mats[fm[hidx]];

        // Texture interpolation (if available)
        int t0 = ft0[hidx], t1 = ft1[hidx], t2 = ft2[hidx];
        float t_u = tx[t0] * hw + tx[t1] * hu + tx[t2] * hv;
        float t_v = ty[t0] * hw + ty[t1] * hu + ty[t2] * hv;
        t_u -= floor(t_u); t_v -= floor(t_v);

        int alb_map = hm.AlbMap;
        int t_w = tw[alb_map];
        int t_h = th[alb_map];
        int t_off = toff[alb_map];

        int t_x = (int)(t_u * t_w);
        int t_y = (int)(t_v * t_h);
        int t_idx = t_off + t_y * t_w + t_x;

        bool hasTxtr = hm.AlbMap > 0;
        float alb_x = tr[t_idx] * hasTxtr + hm.Alb.x * !hasTxtr;
        float alb_y = tg[t_idx] * hasTxtr + hm.Alb.y * !hasTxtr;
        float alb_z = tb[t_idx] * hasTxtr + hm.Alb.z * !hasTxtr;

        // Vertex linear interpolation
        float vrtx_x = RO_x + RD_x * ht;
        float vrtx_y = RO_y + RD_y * ht;
        float vrtx_z = RO_z + RD_z * ht;

        // Normal interpolation
        int n0 = fn0[hidx], n1 = fn1[hidx], n2 = fn2[hidx];
        float nrml_x = nx[n0] * hw + nx[n1] * hu + nx[n2] * hv;
        float nrml_y = ny[n0] * hw + ny[n1] * hu + ny[n2] * hv;
        float nrml_z = nz[n0] * hw + nz[n1] * hu + nz[n2] * hv;
        bool hasNrml = n0 > 0;

        // Calculate the radiance
        float NdotV = RD_x * nrml_x + RD_y * nrml_y + RD_z * nrml_z;
        float lIntensity = ((NdotV * NdotV) * hasNrml + !hasNrml) * hm.Ems.w;
        radi_x += thru_x * hm.Ems.x * alb_x * lIntensity;
        radi_y += thru_y * hm.Ems.y * alb_y * lIntensity;
        radi_z += thru_z * hm.Ems.z * alb_z * lIntensity;

        thru_x *= alb_x * (1.0f - hm.Tr) + hm.Tr;
        thru_y *= alb_y * (1.0f - hm.Tr) + hm.Tr;
        thru_z *= alb_z * (1.0f - hm.Tr) + hm.Tr;

// =================== Indirect lighting =========================

    /* For future me:

    If the surface has a normal:
        We generate a random vector in the hemisphere
        based on cosine weight distribution.
    If the surface does not have a normal:
        We generate a completely random vector in a sphere.
    */

    // Random diffuse lighting
        float rndA = curand_uniform(&rnd[tIdx]);
        float rndB = curand_uniform(&rnd[tIdx]);

        float theta1 = acosf(sqrtf(1.0f - rndA));
        float phi = 2.0f * M_PI * rndB;

        // Cosine weighted hemisphere
        float rnd_x = sinf(theta1) * cosf(phi);
        float rnd_y = sinf(theta1) * sinf(phi);
        float rnd_z = cosf(theta1);

        // For truly random direction
        float theta2 = acosf(1.0f - 2.0f * rndA);
        float truly_rnd_x = sinf(theta2) * cosf(phi);
        float truly_rnd_y = sinf(theta2) * sinf(phi);
        float truly_rnd_z = cosf(theta2);

        // Construct a coordinate system
        bool xGreater = fabsf(nrml_x) > 0.9;
        float ta_x = !xGreater;
        float ta_y = xGreater;

        // Tangent vector
        // There supposed to also be a ta_z, but since its = 0,
        // you can ignore it in the cross product calculation
        float tang_x =  ta_y * nrml_z;
        float tang_y = -ta_x * nrml_z;
        float tang_z = ta_x * nrml_y - ta_y * nrml_x;

        // Bitangent vector
        float bitang_x = tang_y * nrml_z - tang_z * nrml_y;
        float bitang_y = tang_z * nrml_x - tang_x * nrml_z;
        float bitang_z = tang_x * nrml_y - tang_y * nrml_x;

        // Transform the vector to the normal space
        float diff_x = rnd_x * tang_x + rnd_y * bitang_x + rnd_z * nrml_x;
        float diff_y = rnd_x * tang_y + rnd_y * bitang_y + rnd_z * nrml_y;
        float diff_z = rnd_x * tang_z + rnd_y * bitang_z + rnd_z * nrml_z;

    // Specular direction (a.k.a. reflection)
        float spec_x = RD_x - nrml_x * 2.0f * (nrml_x * RD_x);
        float spec_y = RD_y - nrml_y * 2.0f * (nrml_y * RD_y);
        float spec_z = RD_z - nrml_z * 2.0f * (nrml_z * RD_z);

    // Lerp diffuse and specular from roughness/smoothness
        float smooth = 1.0f - hm.Rough;
        float rd_x = diff_x * hm.Rough + spec_x * smooth;
        float rd_y = diff_y * hm.Rough + spec_y * smooth;
        float rd_z = diff_z * hm.Rough + spec_z * smooth;

        bool hasTr = rndA < hm.Tr;
        rd_x = rd_x * !hasTr + RD_x * hasTr;
        rd_y = rd_y * !hasTr + RD_y * hasTr;
        rd_z = rd_z * !hasTr + RD_z * hasTr;

    // Update the ray
        // Origin (truly random for non-normal surfaces)
        RO_x = vrtx_x;
        RO_y = vrtx_y;
        RO_z = vrtx_z;
        // Direction
        RD_x = rd_x * hasNrml + truly_rnd_x * !hasNrml;
        RD_y = rd_y * hasNrml + truly_rnd_y * !hasNrml;
        RD_z = rd_z * hasNrml + truly_rnd_z * !hasNrml;
        // Inverse direction
        RInvd_x = 1.0f / RD_x;
        RInvd_y = 1.0f / RD_y;
        RInvd_z = 1.0f / RD_z;
        // Other ray properties
        RIgnore = hidx;
        RIor = hm.Ior;
    }

    frmbuffer[tIdx] = Flt3(radi_x, radi_y, radi_z);
}
