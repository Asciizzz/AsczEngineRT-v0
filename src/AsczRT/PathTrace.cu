#include <PathTrace.cuh>
#include <AzDevMath.cuh>

// Technically correct
/* However:

Requires importance sampling for the light sources

*/

__global__ void pathtraceKernel(
    AsczCam camera, float *frmx, float *frmy, float *frmz, int frmw, int frmh,
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Geometry data
    int *fv0, int *fv1, int *fv2, int *ft0, int *ft1, int *ft2, int *fn0, int *fn1, int *fn2, int *fm,
    // Materials
    AzMtl *mats, int *lsrc, int lNum,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // BVH data
    float *mi_x, float *mi_y, float *mi_z, float *mx_x, float *mx_y, float *mx_z, int *pl, int *pr, bool *lf, int *gIdx,

    // Additional Debug Data
    curandState *rnd
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= frmw * frmh) return;

    const int MAX_BOUNCES = 5;
    const int MAX_NODES = 64;

    int tX = tIdx % frmw;
    int tY = tIdx / frmw;

    float rnd1 = curand_uniform(&rnd[tIdx]);
    float rnd2 = curand_uniform(&rnd[tIdx]);

    Ray ray = camera.castRay(tX, tY, frmw, frmh, rnd1, rnd2);

    float R_ox  = ray.ox,  R_oy  = ray.oy,  R_oz  = ray.oz;  // Origin
    float R_dx  = ray.dx,  R_dy  = ray.dy,  R_dz  = ray.dz;  // Direction
    float R_rdx = ray.rdx, R_rdy = ray.rdy, R_rdz = ray.rdz; // Inverse direction
    int RIgnore = ray.ignore; // Ignore face index
    float RIor = ray.Ior;     // Index of refraction

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
            float t1n = (mi_x[nidx] - R_ox) * R_rdx;
            float t2n = (mx_x[nidx] - R_ox) * R_rdx;
            float t3n = (mi_y[nidx] - R_oy) * R_rdy;
            float t4n = (mx_y[nidx] - R_oy) * R_rdy;
            float t5n = (mi_z[nidx] - R_oz) * R_rdz;
            float t6n = (mx_z[nidx] - R_oz) * R_rdz;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = R_ox < mi_x[nidx] | R_ox > mx_x[nidx] |
                        R_oy < mi_y[nidx] | R_oy > mx_y[nidx] |
                        R_oz < mi_z[nidx] | R_oz > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > ht) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - R_ox) * R_rdx;
                float t2l = (mx_x[tcl] - R_ox) * R_rdx;
                float t3l = (mi_y[tcl] - R_oy) * R_rdy;
                float t4l = (mx_y[tcl] - R_oy) * R_rdy;
                float t5l = (mi_z[tcl] - R_oz) * R_rdz;
                float t6l = (mx_z[tcl] - R_oz) * R_rdz;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = R_ox < mi_x[tcl] | R_ox > mx_x[tcl] |
                            R_oy < mi_y[tcl] | R_oy > mx_y[tcl] |
                            R_oz < mi_z[tcl] | R_oz > mx_z[tcl];
                float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - R_ox) * R_rdx;
                float t2r = (mx_x[tcr] - R_ox) * R_rdx;
                float t3r = (mi_y[tcr] - R_oy) * R_rdy;
                float t4r = (mx_y[tcr] - R_oy) * R_rdy;
                float t5r = (mi_z[tcr] - R_oz) * R_rdz;
                float t6r = (mx_z[tcr] - R_oz) * R_rdz;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = R_ox < mi_x[tcr] | R_ox > mx_x[tcr] |
                            R_oy < mi_y[tcr] | R_oy > mx_y[tcr] |
                            R_oz < mi_z[tcr] | R_oz > mx_z[tcr];
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

                float hx = R_dy * e2z - R_dz * e2y;
                float hy = R_dz * e2x - R_dx * e2z;
                float hz = R_dx * e2y - R_dy * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a * hit;

                float sx = R_ox - vx[fv0[gi]];
                float sy = R_oy - vy[fv0[gi]];
                float sz = R_oz - vz[fv0[gi]];

                // Since 1/a is used twice and division is expensive
                // Store it in f = 1/a
                float f = 1.0f / a;

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (R_dx * qx + R_dy * qy + R_dz * qz);
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

        if (hidx == -1) {
            // Mess around with these values for fun
            float3 ground = { 0.01f, 0.01f, 0.03f };
            float3 skyHorizon = { 0.01f, 0.01f, 0.03f };
            float3 skyZenith = { 0.00f, 0.00f, 0.00f };
            float3 sunDir = { -1, -1, 1 };
            float sunFocus = 169.0f, sunIntensity = 0.6f;

            // float3 ground = { 1.00f, 1.00f, 1.00f };
            // float3 skyHorizon = { 0.70f, 0.70f, 0.70f };
            // float3 skyZenith = { 0.10f, 0.20f, 0.90f };
            // float3 sunDir = { -1, -1, 1 };
            // float sunFocus = 60.0f, sunIntensity = 8.0f;

            float sunMag = sqrtf(sunDir.x * sunDir.x + sunDir.y * sunDir.y + sunDir.z * sunDir.z);
            sunDir.x /= sunMag; sunDir.y /= sunMag; sunDir.z /= sunMag;

            // Sky calculation
            float sky_t = R_dy * 2.2f;
            sky_t = fmaxf(0.0f, fminf(1.0f, sky_t));
            float skyGradT = powf(sky_t, 0.35f);
            float skyGradR = skyHorizon.x * (1.0f - skyGradT) + skyZenith.x * skyGradT;
            float skyGradG = skyHorizon.y * (1.0f - skyGradT) + skyZenith.y * skyGradT;
            float skyGradB = skyHorizon.z * (1.0f - skyGradT) + skyZenith.z * skyGradT;

            // Sun calculation
            float SdotR = sunDir.x * R_dx + sunDir.y * R_dy + sunDir.z * R_dz;
            SdotR *= -(SdotR < 0.0f);
            float sun_t = powf(SdotR, sunFocus) * sunIntensity;
            bool sun_mask = R_dy > 0.0f;

            // // Star calculation
            // float theta = atan2f(R_dz, R_dx);
            // float phi = acosf(R_dy);

            float final_r = ground.x * !sun_mask + (skyGradR + sun_t) * sun_mask;
            float final_g = ground.y * !sun_mask + (skyGradG + sun_t) * sun_mask;
            float final_b = ground.z * !sun_mask + (skyGradB + sun_t) * sun_mask;

            radi_x += final_r * thru_x;
            radi_y += final_g * thru_y;
            radi_z += final_b * thru_z;

            break;
        }

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
        float alb_x = tr[t_idx] * hasTxtr + hm.Alb_r * !hasTxtr;
        float alb_y = tg[t_idx] * hasTxtr + hm.Alb_g * !hasTxtr;
        float alb_z = tb[t_idx] * hasTxtr + hm.Alb_b * !hasTxtr;

        // Vertex linear interpolation
        float vrtx_x = R_ox + R_dx * ht;
        float vrtx_y = R_oy + R_dy * ht;
        float vrtx_z = R_oz + R_dz * ht;

        // Normal interpolation
        int n0 = fn0[hidx], n1 = fn1[hidx], n2 = fn2[hidx];
        float nrml_x = nx[n0] * hw + nx[n1] * hu + nx[n2] * hv;
        float nrml_y = ny[n0] * hw + ny[n1] * hu + ny[n2] * hv;
        float nrml_z = nz[n0] * hw + nz[n1] * hu + nz[n2] * hv;
        bool hasNrml = n0 > 0;

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
        float spec_x = R_dx - nrml_x * 2.0f * (nrml_x * R_dx);
        float spec_y = R_dy - nrml_y * 2.0f * (nrml_y * R_dy);
        float spec_z = R_dz - nrml_z * 2.0f * (nrml_z * R_dz);

    // Lerp diffuse and specular from roughness/smoothness
        float smooth = 1.0f - hm.Rough;
        float r_dx = diff_x * hm.Rough + spec_x * smooth;
        float r_dy = diff_y * hm.Rough + spec_y * smooth;
        float r_dz = diff_z * hm.Rough + spec_z * smooth;

        float NdotV = nrml_x * R_dx + nrml_y * R_dy + nrml_z * R_dz;
        float Frefl = powf(1.0f - fabs(NdotV), 5.0f);
        float Frefr = 1.0f - Frefl;

        bool hasTr = rndA < hm.Tr * Frefr;
        r_dx = r_dx * !hasTr + R_dx * hasTr;
        r_dy = r_dy * !hasTr + R_dy * hasTr;
        r_dz = r_dz * !hasTr + R_dz * hasTr;

// =================== Direct lighting =========================

        if (lNum == 0) {
            thru_x *= alb_x;
            thru_y *= alb_y;
            thru_z *= alb_z;
            continue;
        }

        // Sample random point on random light source
        int lIdx = lsrc[(int)(lNum * curand_uniform(&rnd[tIdx]))];
        const AzMtl &lm = mats[fm[lIdx]];
        float l_u = curand_uniform(&rnd[tIdx]);
        float l_v = curand_uniform(&rnd[tIdx]);
        bool l_uv_valid = l_u + l_v < 1.0f;
        l_u = l_u * l_uv_valid + (1.0f - l_u) * !l_uv_valid;
        l_v = l_v * l_uv_valid + (1.0f - l_v) * !l_uv_valid;
        float l_w = 1.0f - l_u - l_v;

        int l0 = fv0[lIdx], l1 = fv1[lIdx], l2 = fv2[lIdx];
        float lo_x = vx[l0] * l_u + vx[l1] * l_v + vx[l2] * l_w;
        float lo_y = vy[l0] * l_u + vy[l1] * l_v + vy[l2] * l_w;
        float lo_z = vz[l0] * l_u + vz[l1] * l_v + vz[l2] * l_w;

        float ld_x = vrtx_x - lo_x;
        float ld_y = vrtx_y - lo_y;
        float ld_z = vrtx_z - lo_z;

        float ldistSqr = ld_x * ld_x + ld_y * ld_y + ld_z * ld_z;
        float ldistRsqr = 1.0f / (ldistSqr + !ldistSqr); // Incase ldistSqr = 0
        float ldist = sqrtf(ldistSqr);

        ld_x /= ldist; ld_y /= ldist; ld_z /= ldist;

        float linvd_x = 1.0f / ld_x;
        float linvd_y = 1.0f / ld_y;
        float linvd_z = 1.0f / ld_z;

        // Check for occlusion
        ns_top = 0;
        nstack[ns_top++] = 0;

        bool occluded = false;
        while (ns_top > 0) {
            int nidx = nstack[--ns_top];

            // Check if the ray is outside the bounding box
            float t1n = (mi_x[nidx] - lo_x) * linvd_x;
            float t2n = (mx_x[nidx] - lo_x) * linvd_x;
            float t3n = (mi_y[nidx] - lo_y) * linvd_y;
            float t4n = (mx_y[nidx] - lo_y) * linvd_y;
            float t5n = (mi_z[nidx] - lo_z) * linvd_z;
            float t6n = (mx_z[nidx] - lo_z) * linvd_z;

            float tminn = fminf(t1n, t2n), tmaxn = fmaxf(t1n, t2n);
            tminn = fmaxf(tminn, fminf(t3n, t4n)); tmaxn = fminf(tmaxn, fmaxf(t3n, t4n));
            tminn = fmaxf(tminn, fminf(t5n, t6n)); tmaxn = fminf(tmaxn, fmaxf(t5n, t6n));

            bool nOut = lo_x < mi_x[nidx] | lo_x > mx_x[nidx] |
                        lo_y < mi_y[nidx] | lo_y > mx_y[nidx] |
                        lo_z < mi_z[nidx] | lo_z > mx_z[nidx];
            float nDist = ((tmaxn < tminn | tminn < 0) ? -1 : tminn) * nOut;

            if (nDist < 0 | nDist > ldist) continue;

            // If node is not a leaf:
            if (!lf[nidx]) {
                // Find the distance to the left child
                int tcl = pl[nidx];
                float t1l = (mi_x[tcl] - lo_x) * linvd_x;
                float t2l = (mx_x[tcl] - lo_x) * linvd_x;
                float t3l = (mi_y[tcl] - lo_y) * linvd_y;
                float t4l = (mx_y[tcl] - lo_y) * linvd_y;
                float t5l = (mi_z[tcl] - lo_z) * linvd_z;
                float t6l = (mx_z[tcl] - lo_z) * linvd_z;

                float tminl = fminf(t1l, t2l), tmaxl = fmaxf(t1l, t2l);
                tminl = fmaxf(tminl, fminf(t3l, t4l)); tmaxl = fminf(tmaxl, fmaxf(t3l, t4l));
                tminl = fmaxf(tminl, fminf(t5l, t6l)); tmaxl = fminf(tmaxl, fmaxf(t5l, t6l));

                bool lOut = lo_x < mi_x[tcl] | lo_x > mx_x[tcl] |
                            lo_y < mi_y[tcl] | lo_y > mx_y[tcl] |
                            lo_z < mi_z[tcl] | lo_z > mx_z[tcl];
                float ldist = ((tmaxl < tminl | tminl < 0) ? -1 : tminl) * lOut;

                // Find the distance to the right child
                int tcr = pr[nidx];
                float t1r = (mi_x[tcr] - lo_x) * linvd_x;
                float t2r = (mx_x[tcr] - lo_x) * linvd_x;
                float t3r = (mi_y[tcr] - lo_y) * linvd_y;
                float t4r = (mx_y[tcr] - lo_y) * linvd_y;
                float t5r = (mi_z[tcr] - lo_z) * linvd_z;
                float t6r = (mx_z[tcr] - lo_z) * linvd_z;

                float tminr = fminf(t1r, t2r), tmaxr = fmaxf(t1r, t2r);
                tminr = fmaxf(tminr, fminf(t3r, t4r)); tmaxr = fminf(tmaxr, fmaxf(t3r, t4r));
                tminr = fmaxf(tminr, fminf(t5r, t6r)); tmaxr = fminf(tmaxr, fmaxf(t5r, t6r));

                bool rOut = lo_x < mi_x[tcr] | lo_x > mx_x[tcr] |
                            lo_y < mi_y[tcr] | lo_y > mx_y[tcr] |
                            lo_z < mi_z[tcr] | lo_z > mx_z[tcr];
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

                bool hit = gi != RIgnore & gi != hidx;

                int f0 = fv0[gi], f1 = fv1[gi], f2 = fv2[gi];

                float e1x = vx[f1] - vx[f0];
                float e1y = vy[f1] - vy[f0];
                float e1z = vz[f1] - vz[f0];

                float e2x = vx[f2] - vx[f0];
                float e2y = vy[f2] - vy[f0];
                float e2z = vz[f2] - vz[f0];

                float hx = ld_y * e2z - ld_z * e2y;
                float hy = ld_z * e2x - ld_x * e2z;
                float hz = ld_x * e2y - ld_y * e2x;

                float a = e1x * hx + e1y * hy + e1z * hz;

                hit &= a != 0.0f;
                a = !hit + a;

                float sx = lo_x - vx[f0];
                float sy = lo_y - vy[f0];
                float sz = lo_z - vz[f0];

                // Since 1/a is used twice and division is expensive
                // Store it in f = 1/a
                float f = 1.0f / a;

                float u = f * (sx * hx + sy * hy + sz * hz);

                hit &= u >= 0.0f & u <= 1.0f;

                float qx = sy * e1z - sz * e1y;
                float qy = sz * e1x - sx * e1z;
                float qz = sx * e1y - sy * e1x;

                float v = f * (ld_x * qx + ld_y * qy + ld_z * qz);
                float w = 1.0f - u - v;

                hit &= v >= 0.0f & w >= 0.0f;

                float t = f * (e2x * qx + e2y * qy + e2z * qz);

                hit &= t > 0.0f & t < ldist;

                occluded |= hit;
                ns_top *= !hit;
            }
        }

        occluded = occluded & ldist > EPSILON_2;

        // Calculate the radiance
        float NdotL = nrml_x * ld_x + nrml_y * ld_y + nrml_z * ld_z;
        NdotL *= NdotL + !hasNrml;

        float radi_i = lm.Ems_i * NdotL * !occluded;

        radi_x += thru_x * lm.Ems_r * alb_x * radi_i + hm.Ems_r * hm.Ems_i;
        radi_y += thru_y * lm.Ems_g * alb_y * radi_i + hm.Ems_g * hm.Ems_i;
        radi_z += thru_z * lm.Ems_b * alb_z * radi_i + hm.Ems_b * hm.Ems_i;

        thru_x *= alb_x * (1.0f - hm.Tr) + hm.Tr;
        thru_y *= alb_y * (1.0f - hm.Tr) + hm.Tr;
        thru_z *= alb_z * (1.0f - hm.Tr) + hm.Tr;

// =================== Construct new ray =========================
        // Origin (truly random for non-normal surfaces)
        R_ox = vrtx_x;
        R_oy = vrtx_y;
        R_oz = vrtx_z;
        // Direction
        R_dx = r_dx * hasNrml + truly_rnd_x * !hasNrml;
        R_dy = r_dy * hasNrml + truly_rnd_y * !hasNrml;
        R_dz = r_dz * hasNrml + truly_rnd_z * !hasNrml;
        // Inverse direction
        R_rdx = 1.0f / R_dx;
        R_rdy = 1.0f / R_dy;
        R_rdz = 1.0f / R_dz;
        // Other ray properties
        RIgnore = hidx;
        RIor = hm.Ior;
    }

    frmx[tIdx] = radi_x;
    frmy[tIdx] = radi_y;
    frmz[tIdx] = radi_z;
}
