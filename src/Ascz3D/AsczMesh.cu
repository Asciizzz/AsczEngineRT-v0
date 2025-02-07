#include <AsczMesh.cuh>
#include <cuda_runtime.h>
#include <omp.h>

void AsczMesh::appendMesh(MeshStruct mesh) {
    #pragma omp parallel for
    for (int i = 0; i < mesh.SOrF.size(); ++i) {
        SOrF.push_back(mesh.SOrF[i] + h_fv.size());
    }

    OrSO.push_back(SOrF.size());

    #pragma omp parallel for
    for (int i = 0; i < mesh.geom.size(); ++i) {
        // Offset the indices
        h_fv.push_back(mesh.geom[i].tri.v + h_v.size());
        h_ft.push_back(mesh.geom[i].tri.t + h_t.size());
        h_fn.push_back(mesh.geom[i].tri.n + h_n.size());
        h_fm.push_back(mesh.geom[i].m);
    }

    h_v.insert(h_v.end(), mesh.v.begin(), mesh.v.end());
    h_t.insert(h_t.end(), mesh.t.begin(), mesh.t.end());
    h_n.insert(h_n.end(), mesh.n.begin(), mesh.n.end());

    vNum = h_v.size();
    tNum = h_t.size();
    nNum = h_n.size();
    fNum = h_fv.size();
}

void AsczMesh::freeDevice() {
    if (d_v) { cudaFree(d_v); d_v = nullptr; }
    if (d_t) { cudaFree(d_t); d_t = nullptr; }
    if (d_n) { cudaFree(d_n); d_n = nullptr; }

    if (d_fv) { cudaFree(d_fv); d_fv = nullptr; }
    if (d_ft) { cudaFree(d_ft); d_ft = nullptr; }
    if (d_fn) { cudaFree(d_fn); d_fn = nullptr; }
    if (d_fm) { cudaFree(d_fm); d_fm = nullptr; }
}

void AsczMesh::computeData() {
    h_ABmin.resize(fNum);
    h_ABmax.resize(fNum);
    h_ABcen.resize(fNum);

    #pragma omp parallel for
    for (int i = 0; i < fNum; ++i) {
        Flt3 ABmin = Flt3(INFINITY);
        Flt3 ABmax = Flt3(-INFINITY);

        for (int j = 0; j < 3; ++j) {
            Flt3 v = h_v[h_fv[i][j]];

            ABmin.x = fminf(ABmin.x, v.x);
            ABmin.y = fminf(ABmin.y, v.y);
            ABmin.z = fminf(ABmin.z, v.z);

            ABmax.x = fmaxf(ABmax.x, v.x);
            ABmax.y = fmaxf(ABmax.y, v.y);
            ABmax.z = fmaxf(ABmax.z, v.z);
        }

        h_ABmin[i] = ABmin;
        h_ABmax[i] = ABmax;
        h_ABcen[i] = (ABmin + ABmax) * 0.5f;
    }
}

void AsczMesh::toDevice() {
    freeDevice();
    computeData();

    // -------------------------------------- //    

    cudaMalloc(&d_v, vNum * sizeof(Flt3));
    cudaMalloc(&d_t, tNum * sizeof(Flt2));
    cudaMalloc(&d_n, nNum * sizeof(Flt3));

    cudaMalloc(&d_fv, fNum * sizeof(Int3));
    cudaMalloc(&d_ft, fNum * sizeof(Int3));
    cudaMalloc(&d_fn, fNum * sizeof(Int3));
    cudaMalloc(&d_fm, fNum * sizeof(int));

    cudaMalloc(&d_ABmin, fNum * sizeof(Flt3));
    cudaMalloc(&d_ABmax, fNum * sizeof(Flt3));
    cudaMalloc(&d_ABcen, fNum * sizeof(Flt3));

    // -------------------------------------- //

    cudaMemcpy(d_v, h_v.data(), vNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_t, h_t.data(), tNum * sizeof(Flt2), cudaMemcpyHostToDevice);
    cudaMemcpy(d_n, h_n.data(), nNum * sizeof(Flt3), cudaMemcpyHostToDevice);

    cudaMemcpy(d_fv, h_fv.data(), fNum * sizeof(Int3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ft, h_ft.data(), fNum * sizeof(Int3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fn, h_fn.data(), fNum * sizeof(Int3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fm, h_fm.data(), fNum * sizeof(int), cudaMemcpyHostToDevice);

    cudaMemcpy(d_ABmin, h_ABmin.data(), fNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ABmax, h_ABmax.data(), fNum * sizeof(Flt3), cudaMemcpyHostToDevice);    
    cudaMemcpy(d_ABcen, h_ABcen.data(), fNum * sizeof(Flt3), cudaMemcpyHostToDevice);
}