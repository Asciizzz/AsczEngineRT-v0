#include <AsczMesh.cuh>
#include <cuda_runtime.h>
#include <omp.h>

// Helper function to copy mesh data to device
void fToDevice(VecF &h, float *&d, int size=-1) {
    if (size == -1) size = h.size();
    cudaMalloc(&d, size * sizeof(float));
    cudaMemcpy(d, h.data(), size * sizeof(float), cudaMemcpyHostToDevice);
}

void iToDevice(VecI &h, int *&d, int size=-1) {
    if (size == -1) size = h.size();
    cudaMalloc(&d, size * sizeof(int));
    cudaMemcpy(d, h.data(), size * sizeof(int), cudaMemcpyHostToDevice);
}




void AsczMesh::appendMesh(MeshStruct mesh) {
    #pragma omp parallel for
    for (int i = 0; i < mesh.SOrF.size(); ++i) {
        SOrF.push_back(mesh.SOrF[i] + h_fv0.size());
    }
    OrSO.push_back(SOrF.size());
    oNum++;

    GlbAB.expand(mesh.O_AB);
    O_AB.push_back(mesh.O_AB);
    SO_AB.insert(SO_AB.end(), mesh.SO_AB.begin(), mesh.SO_AB.end());

    int vPrev = h_vx.size();
    int tPrev = h_tx.size();
    int nPrev = h_nx.size();

    for (int i = 0; i < mesh.v.size(); ++i) {
        h_vx.push_back(mesh.v[i].x);
        h_vy.push_back(mesh.v[i].y);
        h_vz.push_back(mesh.v[i].z);
    }
    for (int i = 0; i < mesh.t.size(); ++i) {
        h_tx.push_back(mesh.t[i].x);
        h_ty.push_back(mesh.t[i].y);
    }
    for (int i = 0; i < mesh.n.size(); ++i) {
        h_nx.push_back(mesh.n[i].x);
        h_ny.push_back(mesh.n[i].y);
        h_nz.push_back(mesh.n[i].z);
    }

    for (int i = 0; i < mesh.lSrc.size(); ++i) {
        h_lSrc.push_back(mesh.lSrc[i] + h_fv0.size());
    }

    #pragma omp parallel for
    for (int i = 0; i < mesh.fv.size(); ++i) {
        // Offset the indices and append
        h_fv0.push_back(mesh.fv[i].x + vPrev);
        h_fv1.push_back(mesh.fv[i].y + vPrev);
        h_fv2.push_back(mesh.fv[i].z + vPrev);

        h_ft0.push_back(mesh.ft[i].x + tPrev);
        h_ft1.push_back(mesh.ft[i].y + tPrev);
        h_ft2.push_back(mesh.ft[i].z + tPrev);

        h_fn0.push_back(mesh.fn[i].x + nPrev);
        h_fn1.push_back(mesh.fn[i].y + nPrev);
        h_fn2.push_back(mesh.fn[i].z + nPrev);

        h_fm .push_back(mesh.fm[i]);

        // Get the geometry AABB
        AABB ab;
        ab.expand({ h_vx[h_fv0.back()], h_vy[h_fv0.back()], h_vz[h_fv0.back()] }); 
        ab.expand({ h_vx[h_fv1.back()], h_vy[h_fv1.back()], h_vz[h_fv1.back()] });
        ab.expand({ h_vx[h_fv2.back()], h_vy[h_fv2.back()], h_vz[h_fv2.back()] });

        G_AB.push_back(ab);
    }

    vNum = h_vx.size();
    tNum = h_tx.size();
    nNum = h_nx.size();

    gNum = h_fv0.size();
    lNum = h_lSrc.size();
}

void AsczMesh::freeDevice() {
    if (d_vx) { cudaFree(d_vx); d_vx = nullptr; }
    if (d_vy) { cudaFree(d_vy); d_vy = nullptr; }
    if (d_vz) { cudaFree(d_vz); d_vz = nullptr; }
    if (d_tx) { cudaFree(d_tx); d_tx = nullptr; }
    if (d_ty) { cudaFree(d_ty); d_ty = nullptr; }
    if (d_nx) { cudaFree(d_nx); d_nx = nullptr; }
    if (d_ny) { cudaFree(d_ny); d_ny = nullptr; }
    if (d_nz) { cudaFree(d_nz); d_nz = nullptr; }

    if (d_fv0) { cudaFree(d_fv0); d_fv0 = nullptr; }
    if (d_fv1) { cudaFree(d_fv1); d_fv1 = nullptr; }
    if (d_fv2) { cudaFree(d_fv2); d_fv2 = nullptr; }
    if (d_ft0) { cudaFree(d_ft0); d_ft0 = nullptr; }
    if (d_ft1) { cudaFree(d_ft1); d_ft1 = nullptr; }
    if (d_ft2) { cudaFree(d_ft2); d_ft2 = nullptr; }
    if (d_fn0) { cudaFree(d_fn0); d_fn0 = nullptr; }
    if (d_fn1) { cudaFree(d_fn1); d_fn1 = nullptr; }
    if (d_fn2) { cudaFree(d_fn2); d_fn2 = nullptr; }
    if (d_fm)  { cudaFree(d_fm);  d_fm  = nullptr; }

    if (d_lSrc) { cudaFree(d_lSrc); d_lSrc = nullptr; }
}

void AsczMesh::toDevice() {
    freeDevice();

    fToDevice(h_vx, d_vx); fToDevice(h_vy, d_vy); fToDevice(h_vz, d_vz);
    fToDevice(h_tx, d_tx); fToDevice(h_ty, d_ty);
    fToDevice(h_nx, d_nx); fToDevice(h_ny, d_ny); fToDevice(h_nz, d_nz);

    iToDevice(h_fv0, d_fv0); iToDevice(h_fv1, d_fv1); iToDevice(h_fv2, d_fv2);
    iToDevice(h_ft0, d_ft0); iToDevice(h_ft1, d_ft1); iToDevice(h_ft2, d_ft2);
    iToDevice(h_fn0, d_fn0); iToDevice(h_fn1, d_fn1); iToDevice(h_fn2, d_fn2);
    iToDevice(h_fm,  d_fm);

    iToDevice(h_lSrc, d_lSrc);
}