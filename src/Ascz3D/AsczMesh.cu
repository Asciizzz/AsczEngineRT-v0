#include <AsczMesh.cuh>
#include <cuda_runtime.h>
#include <omp.h>

void AsczMesh::appendMesh(MeshStruct mesh) {
    #pragma omp parallel for
    for (int i = 0; i < mesh.SOrF.size(); ++i) {
        SOrF.push_back(mesh.SOrF[i] + h_geom.size());
    }

    OrSO.push_back(SOrF.size());

    #pragma omp parallel for
    for (int i = 0; i < mesh.geom.size(); ++i) {
        // Offset the indices

        AzGeom &g = mesh.geom[i];
        if (g.type == AzGeom::TRIANGLE) {
            g.tri.v += h_v.size();
            g.tri.t += h_t.size();
            g.tri.n += h_n.size();
        }

        h_geom.push_back(g);
    }

    h_v.insert(h_v.end(), mesh.v.begin(), mesh.v.end());
    h_t.insert(h_t.end(), mesh.t.begin(), mesh.t.end());
    h_n.insert(h_n.end(), mesh.n.begin(), mesh.n.end());

    vNum = h_v.size();
    tNum = h_t.size();
    nNum = h_n.size();

    gNum = h_geom.size();
}

void AsczMesh::freeDevice() {
    if (d_v) { cudaFree(d_v); d_v = nullptr; }
    if (d_t) { cudaFree(d_t); d_t = nullptr; }
    if (d_n) { cudaFree(d_n); d_n = nullptr; }

    if (d_geom) { cudaFree(d_geom); d_geom = nullptr; }
}

void AsczMesh::computeData() {
    h_ABmin.resize(gNum);
    h_ABmax.resize(gNum);
    h_ABcen.resize(gNum);

    #pragma omp parallel for
    for (int i = 0; i < gNum; ++i) {
        Flt3 ABmin = Flt3(INFINITY);
        Flt3 ABmax = Flt3(-INFINITY);
        Flt3 ABcen;

        AzGeom &g = h_geom[i];

        switch (g.type)
        {
        case AzGeom::TRIANGLE:
            for (int j = 0; j < 3; ++j) {
                Flt3 v = h_v[g.tri.v[j]];

                ABmin.x = fminf(ABmin.x, v.x);
                ABmin.y = fminf(ABmin.y, v.y);
                ABmin.z = fminf(ABmin.z, v.z);

                ABmax.x = fmaxf(ABmax.x, v.x);
                ABmax.y = fmaxf(ABmax.y, v.y);
                ABmax.z = fmaxf(ABmax.z, v.z);

                ABcen += v;
            }

            ABcen /= 3;
            break;

        case AzGeom::SPHERE:
            ABmin = g.sph.c - g.sph.r;
            ABmax = g.sph.c + g.sph.r;
            ABcen = g.sph.c;
            break;
        }

        h_ABmin[i] = ABmin;
        h_ABmax[i] = ABmax;
        h_ABcen[i] = ABcen;
    }
}

void AsczMesh::toDevice() {
    freeDevice();
    computeData();

    // -------------------------------------- //    

    cudaMalloc(&d_v, vNum * sizeof(Flt3));
    cudaMalloc(&d_t, tNum * sizeof(Flt2));
    cudaMalloc(&d_n, nNum * sizeof(Flt3));

    cudaMalloc(&d_geom, gNum * sizeof(AzGeom));

    cudaMalloc(&d_ABmin, gNum * sizeof(Flt3));
    cudaMalloc(&d_ABmax, gNum * sizeof(Flt3));
    cudaMalloc(&d_ABcen, gNum * sizeof(Flt3));

    // -------------------------------------- //

    cudaMemcpy(d_v, h_v.data(), vNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_t, h_t.data(), tNum * sizeof(Flt2), cudaMemcpyHostToDevice);
    cudaMemcpy(d_n, h_n.data(), nNum * sizeof(Flt3), cudaMemcpyHostToDevice);

    cudaMemcpy(d_geom, h_geom.data(), gNum * sizeof(AzGeom), cudaMemcpyHostToDevice);

    cudaMemcpy(d_ABmin, h_ABmin.data(), gNum * sizeof(Flt3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ABmax, h_ABmax.data(), gNum * sizeof(Flt3), cudaMemcpyHostToDevice);    
    cudaMemcpy(d_ABcen, h_ABcen.data(), gNum * sizeof(Flt3), cudaMemcpyHostToDevice);
}