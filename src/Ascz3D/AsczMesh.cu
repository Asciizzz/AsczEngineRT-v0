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
    // #pragma omp parallel for
    // for (int i = 0; i < mesh.SOrF.size(); ++i) {
    //     SOrF.push_back(mesh.SOrF[i] + h_fv0.size());
    // }
    // OrSO.push_back(SOrF.size());
    // oNum++;

    // Update global AABB
    GLB_min_x = fminf(GLB_min_x, mesh.O_AB_min_x);
    GLB_min_y = fminf(GLB_min_y, mesh.O_AB_min_y);
    GLB_min_z = fminf(GLB_min_z, mesh.O_AB_min_z);
    GLB_max_x = fmaxf(GLB_max_x, mesh.O_AB_max_x);
    GLB_max_y = fmaxf(GLB_max_y, mesh.O_AB_max_y);
    GLB_max_z = fmaxf(GLB_max_z, mesh.O_AB_max_z);

    int vPrev = h_vx.size();
    int nPrev = h_nx.size();
    int tPrev = h_tx.size();

    h_vx.insert(h_vx.end(), mesh.vx.begin(), mesh.vx.end());
    h_vy.insert(h_vy.end(), mesh.vy.begin(), mesh.vy.end());
    h_vz.insert(h_vz.end(), mesh.vz.begin(), mesh.vz.end());
    h_nx.insert(h_nx.end(), mesh.nx.begin(), mesh.nx.end());
    h_ny.insert(h_ny.end(), mesh.ny.begin(), mesh.ny.end());
    h_nz.insert(h_nz.end(), mesh.nz.begin(), mesh.nz.end());
    h_tx.insert(h_tx.end(), mesh.tx.begin(), mesh.tx.end());
    h_ty.insert(h_ty.end(), mesh.ty.begin(), mesh.ty.end());

    // Append light sources and offset
    #pragma omp parallel
    for (int i = 0; i < mesh.lsrc.size(); ++i) {
        h_lsrc.push_back(mesh.lsrc[i] + h_fv0.size());
    }

    #pragma omp parallel for
    for (int i = 0; i < mesh.fv0.size(); ++i) {
        // Offset the indices and append
        h_fv0.push_back(mesh.fv0[i] + vPrev);
        h_fv1.push_back(mesh.fv1[i] + vPrev);
        h_fv2.push_back(mesh.fv2[i] + vPrev);

        bool hasN = mesh.fn0[i] != -1;
        int offsetN = nPrev * hasN + !hasN;
        h_fn0.push_back(mesh.fn0[i] + offsetN);
        h_fn1.push_back(mesh.fn1[i] + offsetN);
        h_fn2.push_back(mesh.fn2[i] + offsetN);

        bool hasT = mesh.ft0[i] != -1;
        int offsetT = tPrev * hasT + !hasT;
        h_ft0.push_back(mesh.ft0[i] + offsetT);
        h_ft1.push_back(mesh.ft1[i] + offsetT);
        h_ft2.push_back(mesh.ft2[i] + offsetT);

        h_fm .push_back(mesh.fm[i]);

        int fv0 = h_fv0.back();
        int fv1 = h_fv1.back();
        int fv2 = h_fv2.back();

        AB_min_x.push_back(fminf(h_vx[fv0], fminf(h_vx[fv1], h_vx[fv2])));
        AB_min_y.push_back(fminf(h_vy[fv0], fminf(h_vy[fv1], h_vy[fv2])));
        AB_min_z.push_back(fminf(h_vz[fv0], fminf(h_vz[fv1], h_vz[fv2])));
        AB_max_x.push_back(fmaxf(h_vx[fv0], fmaxf(h_vx[fv1], h_vx[fv2])));
        AB_max_y.push_back(fmaxf(h_vy[fv0], fmaxf(h_vy[fv1], h_vy[fv2])));
        AB_max_z.push_back(fmaxf(h_vz[fv0], fmaxf(h_vz[fv1], h_vz[fv2])));

        AB_cx.push_back((AB_min_x.back() + AB_max_x.back()) * 0.5f);
        AB_cy.push_back((AB_min_y.back() + AB_max_y.back()) * 0.5f);
        AB_cz.push_back((AB_min_z.back() + AB_max_z.back()) * 0.5f);
    }

    vNum = h_vx.size();
    tNum = h_tx.size();
    nNum = h_nx.size();

    fNum = h_fv0.size();
    lNum = h_lsrc.size();
}

void AsczMesh::toDevice() {
    ToDevice::F(h_vx, d_vx); ToDevice::F(h_vy, d_vy); ToDevice::F(h_vz, d_vz);
    ToDevice::F(h_nx, d_nx); ToDevice::F(h_ny, d_ny); ToDevice::F(h_nz, d_nz);
    ToDevice::F(h_tx, d_tx); ToDevice::F(h_ty, d_ty);

    ToDevice::I(h_fv0, d_fv0); ToDevice::I(h_fv1, d_fv1); ToDevice::I(h_fv2, d_fv2);
    ToDevice::I(h_fn0, d_fn0); ToDevice::I(h_fn1, d_fn1); ToDevice::I(h_fn2, d_fn2);
    ToDevice::I(h_ft0, d_ft0); ToDevice::I(h_ft1, d_ft1); ToDevice::I(h_ft2, d_ft2);
    ToDevice::I(h_fm,  d_fm);
    ToDevice::I(h_lsrc,d_lsrc);
}