#include <AsczMesh.cuh>
#include <ToDevice.cuh>
#include <omp.h>

AsczMesh::~AsczMesh() {
    if (d_vx) { cudaFree(d_vx); d_vx = nullptr; }
    if (d_vy) { cudaFree(d_vy); d_vy = nullptr; }
    if (d_vz) { cudaFree(d_vz); d_vz = nullptr; }
    if (d_nx) { cudaFree(d_nx); d_nx = nullptr; }
    if (d_ny) { cudaFree(d_ny); d_ny = nullptr; }
    if (d_nz) { cudaFree(d_nz); d_nz = nullptr; }
    if (d_tx) { cudaFree(d_tx); d_tx = nullptr; }
    if (d_ty) { cudaFree(d_ty); d_ty = nullptr; }

    if (d_fv0) { cudaFree(d_fv0); d_fv0 = nullptr; }
    if (d_fv1) { cudaFree(d_fv1); d_fv1 = nullptr; }
    if (d_fv2) { cudaFree(d_fv2); d_fv2 = nullptr; }
    if (d_fn0) { cudaFree(d_fn0); d_fn0 = nullptr; }
    if (d_fn1) { cudaFree(d_fn1); d_fn1 = nullptr; }
    if (d_fn2) { cudaFree(d_fn2); d_fn2 = nullptr; }
    if (d_ft0) { cudaFree(d_ft0); d_ft0 = nullptr; }
    if (d_ft1) { cudaFree(d_ft1); d_ft1 = nullptr; }
    if (d_ft2) { cudaFree(d_ft2); d_ft2 = nullptr; }
    if (d_fm)  { cudaFree(d_fm);  d_fm  = nullptr; }
}

void AsczMesh::append(MeshStruct mesh) {
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
    int nPrev = h_nx.size();
    int tPrev = h_tx.size();

    // Appending vertices, normals, and textures
    #pragma omp parallel
    for (int i = 0; i < mesh.v.size(); ++i) {
        h_vx.push_back(mesh.v[i].x);
        h_vy.push_back(mesh.v[i].y);
        h_vz.push_back(mesh.v[i].z);
    }
    #pragma omp parallel
    for (int i = 0; i < mesh.n.size(); ++i) {
        h_nx.push_back(mesh.n[i].x);
        h_ny.push_back(mesh.n[i].y);
        h_nz.push_back(mesh.n[i].z);
    }
    #pragma omp parallel
    for (int i = 0; i < mesh.t.size(); ++i) {
        h_tx.push_back(mesh.t[i].x);
        h_ty.push_back(mesh.t[i].y);
    }

    // Append light sources and offset
    #pragma omp parallel
    for (int i = 0; i < mesh.lSrc.size(); ++i) {
        h_lsrc.push_back(mesh.lSrc[i] + h_fv0.size());
    }

    #pragma omp parallel for
    for (int i = 0; i < mesh.fv.size(); ++i) {
        // Offset the indices and append
        h_fv0.push_back(mesh.fv[i].x + vPrev);
        h_fv1.push_back(mesh.fv[i].y + vPrev);
        h_fv2.push_back(mesh.fv[i].z + vPrev);

        bool hasN = mesh.fn[i].x != -1;
        int offsetN = nPrev * hasN + !hasN;
        h_fn0.push_back(mesh.fn[i].x + offsetN);
        h_fn1.push_back(mesh.fn[i].y + offsetN);
        h_fn2.push_back(mesh.fn[i].z + offsetN);

        bool hasT = mesh.ft[i].x != -1;
        int offsetT = tPrev * hasT + !hasT;
        h_ft0.push_back(mesh.ft[i].x + offsetT);
        h_ft1.push_back(mesh.ft[i].y + offsetT);
        h_ft2.push_back(mesh.ft[i].z + offsetT);

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
    lNum = mesh.lSrc.size();
}

void AsczMesh::toDevice() {
    ToDevice::F(h_vx, d_vx); ToDevice::F(h_vy, d_vy); ToDevice::F(h_vz, d_vz);
    ToDevice::F(h_nx, d_nx); ToDevice::F(h_ny, d_ny); ToDevice::F(h_nz, d_nz);
    ToDevice::F(h_tx, d_tx); ToDevice::F(h_ty, d_ty);

    ToDevice::I(h_fv0, d_fv0); ToDevice::I(h_fv1, d_fv1); ToDevice::I(h_fv2, d_fv2);
    ToDevice::I(h_fn0, d_fn0); ToDevice::I(h_fn1, d_fn1); ToDevice::I(h_fn2, d_fn2);
    ToDevice::I(h_ft0, d_ft0); ToDevice::I(h_ft1, d_ft1); ToDevice::I(h_ft2, d_ft2);
    ToDevice::I(h_fm,  d_fm);
    ToDevice::I(h_lsrc, d_lsrc);
}