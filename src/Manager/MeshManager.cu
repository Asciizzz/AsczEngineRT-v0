#include <MeshManager.cuh>
#include <cuda_runtime.h>
#include <omp.h>
#include <iostream>

void MeshManager::appendMesh(MeshStruct mesh) {
    #pragma omp parallel for
    for (int i = 0; i < mesh.fv.size(); i++) {
        // Offset the indices
        h_fv.push_back(mesh.fv[i] + h_v.size());
        h_ft.push_back(mesh.ft[i] + h_t.size());
        h_fn.push_back(mesh.fn[i] + h_n.size());
        h_fm.push_back(mesh.fm[i]);
    }

    h_v.insert(h_v.end(), mesh.v.begin(), mesh.v.end());
    h_t.insert(h_t.end(), mesh.t.begin(), mesh.t.end());
    h_n.insert(h_n.end(), mesh.n.begin(), mesh.n.end());

    vNum = h_v.size();
    tNum = h_t.size();
    nNum = h_n.size();
    fNum = h_fv.size();
}

void MeshManager::freeDevice() {
    if (d_v) { cudaFree(d_v); d_v = nullptr; }
    if (d_t) { cudaFree(d_t); d_t = nullptr; }
    if (d_n) { cudaFree(d_n); d_n = nullptr; }

    if (d_fv) { cudaFree(d_fv); d_fv = nullptr; }
    if (d_ft) { cudaFree(d_ft); d_ft = nullptr; }
    if (d_fn) { cudaFree(d_fn); d_fn = nullptr; }
    if (d_fm) { cudaFree(d_fm); d_fm = nullptr; }

    if (d_fmin) { cudaFree(d_fmin); d_fmin = nullptr; } 
    if (d_fmax) { cudaFree(d_fmax); d_fmax = nullptr; }
}

void MeshManager::computeAABB() {
    h_fmin.resize(fNum);
    h_fmax.resize(fNum);

    #pragma omp parallel for
    for (int i = 0; i < fNum; i++) {
        Vec3f minV = Vec3f(INFINITY);
        Vec3f maxV = Vec3f(-INFINITY);

        for (int j = 0; j < 3; j++) {
            Vec3f v = h_v[h_fv[i][j]];
            minV.x = fminf(minV.x, v.x);
            minV.y = fminf(minV.y, v.y);
            minV.z = fminf(minV.z, v.z);    

            maxV.x = fmaxf(maxV.x, v.x);
            maxV.y = fmaxf(maxV.y, v.y);
            maxV.z = fmaxf(maxV.z, v.z);
        }

        h_fmin[i] = minV;
        h_fmax[i] = maxV;
    }
}

void MeshManager::hostToDevice() {
    freeDevice();
    computeAABB();

    // -------------------------------------- //    

    cudaMalloc(&d_v, vNum * sizeof(Vec3f));
    cudaMalloc(&d_t, tNum * sizeof(Vec2f));
    cudaMalloc(&d_n, nNum * sizeof(Vec3f));

    cudaMalloc(&d_fv, fNum * sizeof(Vec3i));
    cudaMalloc(&d_ft, fNum * sizeof(Vec3i));
    cudaMalloc(&d_fn, fNum * sizeof(Vec3i));
    cudaMalloc(&d_fm, fNum * sizeof(int));

    cudaMalloc(&d_fmin, fNum * sizeof(Vec3f));
    cudaMalloc(&d_fmax, fNum * sizeof(Vec3f));

    // -------------------------------------- //

    cudaMemcpy(d_v, h_v.data(), vNum * sizeof(Vec3f), cudaMemcpyHostToDevice);
    cudaMemcpy(d_t, h_t.data(), tNum * sizeof(Vec2f), cudaMemcpyHostToDevice);
    cudaMemcpy(d_n, h_n.data(), nNum * sizeof(Vec3f), cudaMemcpyHostToDevice);

    cudaMemcpy(d_fv, h_fv.data(), fNum * sizeof(Vec3i), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ft, h_ft.data(), fNum * sizeof(Vec3i), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fn, h_fn.data(), fNum * sizeof(Vec3i), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fm, h_fm.data(), fNum * sizeof(int), cudaMemcpyHostToDevice);

    cudaMemcpy(d_fmin, h_fmin.data(), fNum * sizeof(Vec3f), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fmax, h_fmax.data(), fNum * sizeof(Vec3f), cudaMemcpyHostToDevice);
}