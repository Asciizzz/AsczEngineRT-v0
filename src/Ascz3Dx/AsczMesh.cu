#include <AsczMesh.cuh>
#include <cuda_runtime.h>
#include <omp.h>

int AsczMesh::appendVrtx(Flt3 v) {
    h_v.push_back(v); return vNum++;
}
int AsczMesh::appendTxtr(Flt2 t) {
    h_t.push_back(t); return tNum++;
}
int AsczMesh::appendNrml(Flt3 n) {
    h_n.push_back(n); return nNum++;
}



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

    int vPrev = h_v.size();
    int tPrev = h_t.size();
    int nPrev = h_n.size();

    h_v.insert(h_v.end(), mesh.v.begin(), mesh.v.end());
    h_t.insert(h_t.end(), mesh.t.begin(), mesh.t.end());
    h_n.insert(h_n.end(), mesh.n.begin(), mesh.n.end());

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

            ab.expand(h_v[g.tri.v[0]]);
            ab.expand(h_v[g.tri.v[1]]);
            ab.expand(h_v[g.tri.v[2]]);
        }
        else if (g.type == AzGeom::SPHERE) {
            g.sph.c += vPrev;

            Flt3 c = h_v[g.sph.c];
            ab.min = c - g.sph.r;
            ab.max = c + g.sph.r;
        }

        h_geom.push_back(g);
        G_AB.push_back(ab);
    }

    vNum = h_v.size();
    tNum = h_t.size();
    nNum = h_n.size();

    gNum = h_geom.size();
    lNum = h_lSrc.size();
}

void AsczMesh::freeDevice() {
    if (d_v) { cudaFree(d_v); d_v = nullptr; }
    if (d_t) { cudaFree(d_t); d_t = nullptr; }
    if (d_n) { cudaFree(d_n); d_n = nullptr; }

    if (d_geom) { cudaFree(d_geom); d_geom = nullptr; }
    if (d_lSrc) { cudaFree(d_lSrc); d_lSrc = nullptr; }
}

void AsczMesh::toDevice() {
    freeDevice();

    cudaMalloc(&d_v, vNum * sizeof(Flt3));
    cudaMalloc(&d_t, tNum * sizeof(Flt2));
    cudaMalloc(&d_n, nNum * sizeof(Flt3));
    cudaMalloc(&d_geom, gNum * sizeof(AzGeom));
    cudaMalloc(&d_lSrc, h_lSrc.size() * sizeof(int));

    cudaMemcpy(d_v, h_v.data(), vNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_t, h_t.data(), tNum * sizeof(Flt2), cudaMemcpyHostToDevice);
    cudaMemcpy(d_n, h_n.data(), nNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_geom, h_geom.data(), gNum * sizeof(AzGeom), cudaMemcpyHostToDevice);
    cudaMemcpy(d_lSrc, h_lSrc.data(), h_lSrc.size() * sizeof(int), cudaMemcpyHostToDevice);
}