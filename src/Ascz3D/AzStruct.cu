#include <AzStruct.cuh>

#include <omp.h>

#include <fstream>
#include <iostream>
#include <unordered_map>

#include <stb_image.h>
#include <cuda_runtime.h>

#include <ToDevice.cuh>

void D_AzMesh::copy(AzMesh &ms) {
    ToDevice::F(ms.vx, vx);
    ToDevice::F(ms.vy, vy);
    ToDevice::F(ms.vz, vz);

    ToDevice::F(ms.nx, nx);
    ToDevice::F(ms.ny, ny);
    ToDevice::F(ms.nz, nz);

    ToDevice::F(ms.tx, tx);
    ToDevice::F(ms.ty, ty);

    ToDevice::I(ms.fv0, fv0);
    ToDevice::I(ms.fv1, fv1);
    ToDevice::I(ms.fv2, fv2);

    ToDevice::I(ms.fn0, fn0);
    ToDevice::I(ms.fn1, fn1);
    ToDevice::I(ms.fn2, fn2);

    ToDevice::I(ms.ft0, ft0);
    ToDevice::I(ms.ft1, ft1);
    ToDevice::I(ms.ft2, ft2);

    ToDevice::I(ms.fm, fm);
    ToDevice::I(ms.lsrc, lsrc);

    v_num = ms.v_num;
    n_num = ms.n_num;
    t_num = ms.t_num;
    f_num = ms.f_num;
    l_num = ms.l_num;
}



int AzMtl::push() {
    Alb_r.push_back(1.0f);
    Alb_g.push_back(1.0f);
    Alb_b.push_back(1.0f);
    AlbMap. push_back(-1);

    Rough.push_back(1.0f);
    Metal.push_back(0.0f);

    Tr.   push_back(0.0f);
    Ior.  push_back(1.0f);

    Ems_r.push_back(0.0f);
    Ems_g.push_back(0.0f);
    Ems_b.push_back(0.0f);
    Ems_i.push_back(0.0f);

    return num++;
}

void D_AzMtl::copy(AzMtl &mt) {
    ToDevice::F(mt.Alb_r, Alb_r);
    ToDevice::F(mt.Alb_g, Alb_g);
    ToDevice::F(mt.Alb_b, Alb_b);
    ToDevice::I(mt.AlbMap, AlbMap);

    ToDevice::F(mt.Rough, Rough);
    ToDevice::F(mt.Metal, Metal);

    ToDevice::F(mt.Tr, Tr);
    ToDevice::F(mt.Ior, Ior);

    ToDevice::F(mt.Ems_r, Ems_r);
    ToDevice::F(mt.Ems_g, Ems_g);
    ToDevice::F(mt.Ems_b, Ems_b);
    ToDevice::F(mt.Ems_i, Ems_i);

    num = mt.num;
}



int AzTxtr::append(const char *path) {
    int tw, th, tn;
    unsigned char *data = stbi_load(path, &tw, &th, &tn, 4);

    if (data == nullptr) return 0;

    w.push_back(tw);
    h.push_back(th);
    off.push_back(size);

    float _255 = 1.0f / 255.0f; // Division is expensive

    for (int y = th - 1; y > -1; y--) {
        for (int x = 0; x < tw; x++) {
            int i = (y * tw + x) * 4;

            r.push_back(data[i + 0] * _255);
            g.push_back(data[i + 1] * _255);
            b.push_back(data[i + 2] * _255);
            a.push_back(data[i + 3] * _255);
        }
    }

    size += tw * th;

    return num++;
}

void D_AzTxtr::copy(AzTxtr &tx) {
    ToDevice::F(tx.r, r);
    ToDevice::F(tx.g, g);
    ToDevice::F(tx.b, b);
    ToDevice::F(tx.a, a);

    ToDevice::I(tx.w, w);
    ToDevice::I(tx.h, h);
    ToDevice::I(tx.off, off);

    size = tx.size;
    num = tx.num;
}



AzGlobal::AzGlobal() {
    // Default vertex
    MS.vx.push_back(0.0f); MS.vy.push_back(0.0f); MS.vz.push_back(0.0f);
    MS.nx.push_back(0.0f); MS.ny.push_back(0.0f); MS.nz.push_back(0.0f);
    MS.tx.push_back(0.0f); MS.ty.push_back(0.0f);

    MS.fv0.push_back(0); MS.fv1.push_back(0); MS.fv2.push_back(0);
    MS.fn0.push_back(0); MS.fn1.push_back(0); MS.fn2.push_back(0);
    MS.ft0.push_back(0); MS.ft1.push_back(0); MS.ft2.push_back(0);

    MS.fm.push_back(0);
    MS.lsrc.push_back(0);

    // Default material
    MT.push();
    MT.AlbMap[0] = 0;

    // Default texture
    TX.r.push_back(0.0f); TX.g.push_back(0.0f); TX.b.push_back(0.0f); TX.a.push_back(0.0f);
    TX.w.push_back(1); TX.h.push_back(1); TX.off.push_back(0);

    // Update count
    MS.v_num = 1; MS.n_num = 1; MS.t_num = 1;
    MS.f_num = 1; MS.l_num = 1;
    MT.num = 1; TX.num = 1; TX.size = 1;
}

void AzGlobal::copy() {
    d_MS.copy(MS);
    d_MT.copy(MT);
    d_TX.copy(TX);
}

void AzGlobal::gulp(AzObj &obj) {
// Important: This does not include the AABB and centroid data

// GET OLD COUNTS
    int MS_v_num = MS.v_num;
    int MS_n_num = MS.n_num;
    int MS_t_num = MS.t_num;

    int TX_size = TX.size;
    int TX_num = TX.num;

    int MT_num = MT.num;

// COMBINE TEXTURES
    TX.r.insert(TX.r.end(), obj.TX.r.begin(), obj.TX.r.end());
    TX.g.insert(TX.g.end(), obj.TX.g.begin(), obj.TX.g.end());
    TX.b.insert(TX.b.end(), obj.TX.b.begin(), obj.TX.b.end());
    TX.a.insert(TX.a.end(), obj.TX.a.begin(), obj.TX.a.end());

    TX.w.insert(TX.w.end(), obj.TX.w.begin(), obj.TX.w.end());
    TX.h.insert(TX.h.end(), obj.TX.h.begin(), obj.TX.h.end());

    // Offset the texture based on existing textures size
    for (int i = 0; i < obj.TX.num; ++i) {
        TX.off.push_back(obj.TX.off[i] + TX_size);
    }

// COMBINE MATERIALS
    MT.Alb_r.insert(MT.Alb_r.end(), obj.MT.Alb_r.begin(), obj.MT.Alb_r.end());
    MT.Alb_g.insert(MT.Alb_g.end(), obj.MT.Alb_g.begin(), obj.MT.Alb_g.end());
    MT.Alb_b.insert(MT.Alb_b.end(), obj.MT.Alb_b.begin(), obj.MT.Alb_b.end());

    // Offset albedo map based on existing textures size
    for (int i = 0; i < obj.MT.num; ++i) {
        int am = obj.MT.AlbMap[i];
        MT.AlbMap.push_back((am > -1) * (am + TX_num));
    }

    MT.Rough.insert(MT.Rough.end(), obj.MT.Rough.begin(), obj.MT.Rough.end());
    MT.Metal.insert(MT.Metal.end(), obj.MT.Metal.begin(), obj.MT.Metal.end());

    MT.Tr.insert(MT.Tr.end(), obj.MT.Tr.begin(), obj.MT.Tr.end());
    MT.Ior.insert(MT.Ior.end(), obj.MT.Ior.begin(), obj.MT.Ior.end());

    MT.Ems_r.insert(MT.Ems_r.end(), obj.MT.Ems_r.begin(), obj.MT.Ems_r.end());
    MT.Ems_g.insert(MT.Ems_g.end(), obj.MT.Ems_g.begin(), obj.MT.Ems_g.end());
    MT.Ems_b.insert(MT.Ems_b.end(), obj.MT.Ems_b.begin(), obj.MT.Ems_b.end());
    MT.Ems_i.insert(MT.Ems_i.end(), obj.MT.Ems_i.begin(), obj.MT.Ems_i.end());

// COMBINE MESHES
    MS.vx.insert(MS.vx.end(), obj.MS.vx.begin(), obj.MS.vx.end());
    MS.vy.insert(MS.vy.end(), obj.MS.vy.begin(), obj.MS.vy.end());
    MS.vz.insert(MS.vz.end(), obj.MS.vz.begin(), obj.MS.vz.end());
    MS.nx.insert(MS.nx.end(), obj.MS.nx.begin(), obj.MS.nx.end());
    MS.ny.insert(MS.ny.end(), obj.MS.ny.begin(), obj.MS.ny.end());
    MS.nz.insert(MS.nz.end(), obj.MS.nz.begin(), obj.MS.nz.end());
    MS.tx.insert(MS.tx.end(), obj.MS.tx.begin(), obj.MS.tx.end());
    MS.ty.insert(MS.ty.end(), obj.MS.ty.begin(), obj.MS.ty.end());

    // Append light sources and offset
    #pragma omp parallel for
    for (int lsrc : obj.MS.lsrc) {
        MS.lsrc.push_back(lsrc + MS.f_num);
    }

    // Append other faces data and offset
    #pragma omp parallel for
    for (int i = 0; i < obj.MS.f_num; ++i) {
        MS.fv0.push_back(obj.MS.fv0[i] + MS_v_num);
        MS.fv1.push_back(obj.MS.fv1[i] + MS_v_num);
        MS.fv2.push_back(obj.MS.fv2[i] + MS_v_num);

        bool hasN = obj.MS.fn0[i] != -1;
        int offsetN = MS_n_num * hasN + !hasN;
        MS.fn0.push_back(obj.MS.fn0[i] + offsetN);
        MS.fn1.push_back(obj.MS.fn1[i] + offsetN);
        MS.fn2.push_back(obj.MS.fn2[i] + offsetN);

        bool hasT = obj.MS.ft0[i] != -1;
        int offsetT = MS_t_num * hasT + !hasT;
        MS.ft0.push_back(obj.MS.ft0[i] + offsetT);
        MS.ft1.push_back(obj.MS.ft1[i] + offsetT);
        MS.ft2.push_back(obj.MS.ft2[i] + offsetT);

        bool hasM = obj.MS.fm[i] != -1;
        int offsetM = MT_num * hasM + !hasM;
        MS.fm.push_back(obj.MS.fm[i] + offsetM);
    }

    // Update counts
    MS.v_num += obj.MS.v_num;
    MS.n_num += obj.MS.n_num;
    MS.t_num += obj.MS.t_num;
    MS.f_num += obj.MS.f_num;
    MS.l_num += obj.MS.l_num;

    TX.num += obj.TX.num;
    TX.size += obj.TX.size;

    MT.num += obj.MT.num;
}


__device__ float _3 = 1.0f / 3.0f;
__global__ void computeAABB(
    float *min_x, float *min_y, float *min_z,
    float *max_x, float *max_y, float *max_z,
    float *cx, float *cy, float *cz,

    float *vx, float *vy, float *vz,
    int *fv0, int *fv1, int *fv2,
    int fNum
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= fNum) return;

    int fv0_idx = fv0[idx];
    int fv1_idx = fv1[idx];
    int fv2_idx = fv2[idx];

    float v0x = vx[fv0_idx], v0y = vy[fv0_idx], v0z = vz[fv0_idx];
    float v1x = vx[fv1_idx], v1y = vy[fv1_idx], v1z = vz[fv1_idx];
    float v2x = vx[fv2_idx], v2y = vy[fv2_idx], v2z = vz[fv2_idx];

    min_x[idx] = fminf(v0x, fminf(v1x, v2x));
    min_y[idx] = fminf(v0y, fminf(v1y, v2y));
    min_z[idx] = fminf(v0z, fminf(v1z, v2z));

    max_x[idx] = fmaxf(v0x, fmaxf(v1x, v2x));
    max_y[idx] = fmaxf(v0y, fmaxf(v1y, v2y));
    max_z[idx] = fmaxf(v0z, fmaxf(v1z, v2z));

    cx[idx] = (v0x + v1x + v2x) * _3;
    cy[idx] = (v0y + v1y + v2y) * _3;
    cz[idx] = (v0z + v1z + v2z) * _3;
}

void AzGlobal::computeAB() {
    cudaMalloc(&d_MS.min_x, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.min_y, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.min_z, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.max_x, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.max_y, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.max_z, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.fcx, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.fcy, MS.f_num * sizeof(float));
    cudaMalloc(&d_MS.fcz, MS.f_num * sizeof(float));

    computeAABB<<<(MS.f_num + 255) / 256, 256>>>(
        d_MS.min_x, d_MS.min_y, d_MS.min_z,
        d_MS.max_x, d_MS.max_y, d_MS.max_z,
        d_MS.fcx, d_MS.fcy, d_MS.fcz,
        d_MS.vx, d_MS.vy, d_MS.vz,
        d_MS.fv0, d_MS.fv1, d_MS.fv2,
        MS.f_num
    );

    // Copy back to host
    MS.min_x = new float[MS.f_num];
    MS.min_y = new float[MS.f_num];
    MS.min_z = new float[MS.f_num];
    MS.max_x = new float[MS.f_num];
    MS.max_y = new float[MS.f_num];
    MS.max_z = new float[MS.f_num];
    MS.fcx = new float[MS.f_num];
    MS.fcy = new float[MS.f_num];
    MS.fcz = new float[MS.f_num];

    cudaMemcpy(MS.min_x, d_MS.min_x, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.min_y, d_MS.min_y, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.min_z, d_MS.min_z, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.max_x, d_MS.max_x, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.max_y, d_MS.max_y, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.max_z, d_MS.max_z, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.fcx, d_MS.fcx, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.fcy, d_MS.fcy, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(MS.fcz, d_MS.fcz, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
}