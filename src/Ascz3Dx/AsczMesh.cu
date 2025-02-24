#include <AsczMesh.cuh>
#include <cuda_runtime.h>
#include <omp.h>


void AsczMesh::appendMesh(MeshStruct mesh) {
    #pragma omp parallel for
    for (int i = 0; i < mesh.SOrF.size(); ++i) {
        SOrF.push_back(mesh.SOrF[i] + h_geom.size());
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
        h_lSrc.push_back(mesh.lSrc[i] + h_geom.size());
    }

    #pragma omp parallel for
    for (int i = 0; i < mesh.geom.size(); ++i) {
        AABB ab;

        AzGeom &g = mesh.geom[i];
        if (g.type == AzGeom::TRIANGLE) {
            g.tri.v += vPrev;
            g.tri.t += tPrev;
            g.tri.n += nPrev;

            Flt3 v0 = { h_vx[g.tri.v[0]], h_vy[g.tri.v[0]], h_vz[g.tri.v[0]] };
            Flt3 v1 = { h_vx[g.tri.v[1]], h_vy[g.tri.v[1]], h_vz[g.tri.v[1]] };
            Flt3 v2 = { h_vx[g.tri.v[2]], h_vy[g.tri.v[2]], h_vz[g.tri.v[2]] };

            ab.expand(v0);
            ab.expand(v1);
            ab.expand(v2);
        }
        else if (g.type == AzGeom::SPHERE) {
            g.sph.c += vPrev;

            Flt3 c = { h_vx[g.sph.c], h_vy[g.sph.c], h_vz[g.sph.c] };
            ab.min = c - g.sph.r;
            ab.max = c + g.sph.r;
        }

        h_geom.push_back(g);
        G_AB.push_back(ab);
    }

    vNum = h_vx.size();
    tNum = h_tx.size();
    nNum = h_nx.size();

    gNum = h_geom.size();
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

    if (d_geom) { cudaFree(d_geom); d_geom = nullptr; }
    if (d_lSrc) { cudaFree(d_lSrc); d_lSrc = nullptr; }
}

void AsczMesh::toDevice() {
    freeDevice();

    cudaMalloc(&d_vx, vNum * sizeof(float));
    cudaMalloc(&d_vy, vNum * sizeof(float));
    cudaMalloc(&d_vz, vNum * sizeof(float));
    cudaMalloc(&d_tx, tNum * sizeof(float));
    cudaMalloc(&d_ty, tNum * sizeof(float));
    cudaMalloc(&d_nx, nNum * sizeof(float));
    cudaMalloc(&d_ny, nNum * sizeof(float));
    cudaMalloc(&d_nz, nNum * sizeof(float));

    cudaMemcpy(d_vx, h_vx.data(), vNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vy, h_vy.data(), vNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vz, h_vz.data(), vNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_tx, h_tx.data(), tNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ty, h_ty.data(), tNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nx, h_nx.data(), nNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ny, h_ny.data(), nNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nz, h_nz.data(), nNum * sizeof(float), cudaMemcpyHostToDevice);

    cudaMalloc(&d_geom, gNum * sizeof(AzGeom));
    cudaMalloc(&d_lSrc, h_lSrc.size() * sizeof(int));

    cudaMemcpy(d_geom, h_geom.data(), gNum * sizeof(AzGeom), cudaMemcpyHostToDevice);
    cudaMemcpy(d_lSrc, h_lSrc.data(), h_lSrc.size() * sizeof(int), cudaMemcpyHostToDevice);
}