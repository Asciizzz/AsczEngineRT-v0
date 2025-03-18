#define STB_IMAGE_IMPLEMENTATION

#include <AzStruct.cuh>
#include <omp.h>

#include <ostream>
#include <iostream>
#include <fstream>

#include <ToDevice.cuh>

void D_AzMesh::copy(AzMesh &ms) {
    ToDevice::F(ms.vx, vx); ToDevice::F(ms.vy, vy); ToDevice::F(ms.vz, vz);
    ToDevice::F(ms.nx, nx); ToDevice::F(ms.ny, ny); ToDevice::F(ms.nz, nz);
    ToDevice::F(ms.tx, tx); ToDevice::F(ms.ty, ty);

    ToDevice::I(ms.fv0, fv0); ToDevice::I(ms.fv1, fv1); ToDevice::I(ms.fv2, fv2);
    ToDevice::I(ms.fn0, fn0); ToDevice::I(ms.fn1, fn1); ToDevice::I(ms.fn2, fn2);
    ToDevice::I(ms.ft0, ft0); ToDevice::I(ms.ft1, ft1); ToDevice::I(ms.ft2, ft2);

    ToDevice::I(ms.fm, fm); ToDevice::I(ms.lsrc, lsrc);

    v_num = ms.v_num;
    n_num = ms.n_num;
    t_num = ms.t_num;
    f_num = ms.f_num;
    l_num = ms.l_num;
}

void D_AzMesh::free() {
    cudaFree(vx); cudaFree(vy); cudaFree(vz);
    cudaFree(nx); cudaFree(ny); cudaFree(nz);
    cudaFree(tx); cudaFree(ty);

    cudaFree(fv0); cudaFree(fv1); cudaFree(fv2);
    cudaFree(fn0); cudaFree(fn1); cudaFree(fn2);
    cudaFree(ft0); cudaFree(ft1); cudaFree(ft2);

    cudaFree(fm); cudaFree(lsrc);
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

void D_AzMtl::free() {
    cudaFree(Alb_r); cudaFree(Alb_g); cudaFree(Alb_b);
    cudaFree(AlbMap);

    cudaFree(Rough); cudaFree(Metal);
    cudaFree(Tr); cudaFree(Ior);

    cudaFree(Ems_r); cudaFree(Ems_g); cudaFree(Ems_b); cudaFree(Ems_i);
}



int AzTxtr::append(const char *path) {
    int tw, th, tn;
    unsigned char *data = stbi_load(path, &tw, &th, &tn, 4);

    if (data == nullptr) return -1;

    w.push_back(tw);
    h.push_back(th);
    off.push_back(size);
    size += tw * th;

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

    std::cout << tw << "x" << th << " ... ";

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

void D_AzTxtr::free() {
    cudaFree(r); cudaFree(g); cudaFree(b); cudaFree(a);
    cudaFree(w); cudaFree(h); cudaFree(off);
}


void AzObj::save(AzObj &Obj, const char *path) {
    std::ofstream file(path, std::ios::binary);

    auto write_vec = [&](auto& vec) {
        size_t size = vec.size();
        file.write(reinterpret_cast<const char*>(&size), sizeof(size));
        file.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(typename std::decay_t<decltype(vec)>::value_type));
    };

    // Write mesh data
    write_vec(Obj.MS.vx); write_vec(Obj.MS.vy); write_vec(Obj.MS.vz);
    write_vec(Obj.MS.nx); write_vec(Obj.MS.ny); write_vec(Obj.MS.nz);
    write_vec(Obj.MS.tx); write_vec(Obj.MS.ty);

    write_vec(Obj.MS.fv0); write_vec(Obj.MS.fv1); write_vec(Obj.MS.fv2);
    write_vec(Obj.MS.fn0); write_vec(Obj.MS.fn1); write_vec(Obj.MS.fn2);
    write_vec(Obj.MS.ft0); write_vec(Obj.MS.ft1); write_vec(Obj.MS.ft2);

    write_vec(Obj.MS.fm);
    write_vec(Obj.MS.lsrc);

    file.write(reinterpret_cast<const char *>(&Obj.MS.v_num), sizeof(Obj.MS.v_num));
    file.write(reinterpret_cast<const char *>(&Obj.MS.n_num), sizeof(Obj.MS.n_num));
    file.write(reinterpret_cast<const char *>(&Obj.MS.t_num), sizeof(Obj.MS.t_num));
    file.write(reinterpret_cast<const char *>(&Obj.MS.f_num), sizeof(Obj.MS.f_num));
    file.write(reinterpret_cast<const char *>(&Obj.MS.l_num), sizeof(Obj.MS.l_num));

    // Write material data
    write_vec(Obj.MT.Alb_r); write_vec(Obj.MT.Alb_g); write_vec(Obj.MT.Alb_b);
    write_vec(Obj.MT.AlbMap);
    
    write_vec(Obj.MT.Rough); write_vec(Obj.MT.Metal);
    write_vec(Obj.MT.Tr); write_vec(Obj.MT.Ior);

    write_vec(Obj.MT.Ems_r); write_vec(Obj.MT.Ems_g); write_vec(Obj.MT.Ems_b); write_vec(Obj.MT.Ems_i);

    file.write(reinterpret_cast<const char *>(&Obj.MT.num), sizeof(Obj.MT.num));

    // Write texture data
    write_vec(Obj.TX.r); write_vec(Obj.TX.g); write_vec(Obj.TX.b); write_vec(Obj.TX.a);
    write_vec(Obj.TX.w); write_vec(Obj.TX.h); write_vec(Obj.TX.off);

    file.write(reinterpret_cast<const char *>(&Obj.TX.size), sizeof(Obj.TX.size));
    file.write(reinterpret_cast<const char *>(&Obj.TX.num), sizeof(Obj.TX.num));

    // Write AABB data
    file.write(reinterpret_cast<const char *>(&Obj.AB_x), sizeof(Obj.AB_x));
    file.write(reinterpret_cast<const char *>(&Obj.AB_y), sizeof(Obj.AB_y));
    file.write(reinterpret_cast<const char *>(&Obj.AB_z), sizeof(Obj.AB_z));
    file.write(reinterpret_cast<const char *>(&Obj.AB_X), sizeof(Obj.AB_X));
    file.write(reinterpret_cast<const char *>(&Obj.AB_Y), sizeof(Obj.AB_Y));
    file.write(reinterpret_cast<const char *>(&Obj.AB_Z), sizeof(Obj.AB_Z));

    file.close();
}

AzObj AzObj::load(const char *path) {
    std::ifstream file(path, std::ios::binary);

    if (!file) {
        std::cerr << "Error opening file for reading: " << path << std::endl;
        return AzObj();
    }

    auto read_vec = [&](auto& vec) {
        size_t size;
        file.read(reinterpret_cast<char*>(&size), sizeof(size));
        vec.resize(size);
        file.read(reinterpret_cast<char*>(vec.data()), size * sizeof(typename std::decay_t<decltype(vec)>::value_type));
    };

    AzObj obj;

    // Load mesh data
    read_vec(obj.MS.vx); read_vec(obj.MS.vy); read_vec(obj.MS.vz);
    read_vec(obj.MS.nx); read_vec(obj.MS.ny); read_vec(obj.MS.nz);
    read_vec(obj.MS.tx); read_vec(obj.MS.ty);

    read_vec(obj.MS.fv0); read_vec(obj.MS.fv1); read_vec(obj.MS.fv2);
    read_vec(obj.MS.fn0); read_vec(obj.MS.fn1); read_vec(obj.MS.fn2);
    read_vec(obj.MS.ft0); read_vec(obj.MS.ft1); read_vec(obj.MS.ft2);

    read_vec(obj.MS.fm);
    read_vec(obj.MS.lsrc);

    file.read(reinterpret_cast<char *>(&obj.MS.v_num), sizeof(obj.MS.v_num));
    file.read(reinterpret_cast<char *>(&obj.MS.n_num), sizeof(obj.MS.n_num));
    file.read(reinterpret_cast<char *>(&obj.MS.t_num), sizeof(obj.MS.t_num));
    file.read(reinterpret_cast<char *>(&obj.MS.f_num), sizeof(obj.MS.f_num));
    file.read(reinterpret_cast<char *>(&obj.MS.l_num), sizeof(obj.MS.l_num));

    // Load material data
    read_vec(obj.MT.Alb_r); read_vec(obj.MT.Alb_g); read_vec(obj.MT.Alb_b);
    read_vec(obj.MT.AlbMap);

    read_vec(obj.MT.Rough); read_vec(obj.MT.Metal);
    read_vec(obj.MT.Tr); read_vec(obj.MT.Ior);

    read_vec(obj.MT.Ems_r); read_vec(obj.MT.Ems_g); read_vec(obj.MT.Ems_b); read_vec(obj.MT.Ems_i);

    file.read(reinterpret_cast<char *>(&obj.MT.num), sizeof(obj.MT.num));

    // Load texture data
    read_vec(obj.TX.r); read_vec(obj.TX.g); read_vec(obj.TX.b); read_vec(obj.TX.a);
    read_vec(obj.TX.w); read_vec(obj.TX.h); read_vec(obj.TX.off);

    file.read(reinterpret_cast<char *>(&obj.TX.size), sizeof(obj.TX.size));
    file.read(reinterpret_cast<char *>(&obj.TX.num), sizeof(obj.TX.num));

    // Load AABB data
    file.read(reinterpret_cast<char *>(&obj.AB_x), sizeof(obj.AB_x));
    file.read(reinterpret_cast<char *>(&obj.AB_y), sizeof(obj.AB_y));
    file.read(reinterpret_cast<char *>(&obj.AB_z), sizeof(obj.AB_z));
    file.read(reinterpret_cast<char *>(&obj.AB_X), sizeof(obj.AB_X));
    file.read(reinterpret_cast<char *>(&obj.AB_Y), sizeof(obj.AB_Y));
    file.read(reinterpret_cast<char *>(&obj.AB_Z), sizeof(obj.AB_Z));

    file.close();

    return obj;
}

void AzObj::combine(AzObj &obj) {
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
        MT.AlbMap.push_back(am > -1 ? am + TX_num : -1);
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

        bool hasN = obj.MS.fn0[i] > -1;
        MS.fn0.push_back(hasN ? obj.MS.fn0[i] + MS_n_num : -1);
        MS.fn1.push_back(hasN ? obj.MS.fn1[i] + MS_n_num : -1);
        MS.fn2.push_back(hasN ? obj.MS.fn2[i] + MS_n_num : -1);

        bool hasT = obj.MS.ft0[i] > -1;
        MS.ft0.push_back(hasT ? obj.MS.ft0[i] + MS_t_num : -1);
        MS.ft1.push_back(hasT ? obj.MS.ft1[i] + MS_t_num : -1);
        MS.ft2.push_back(hasT ? obj.MS.ft2[i] + MS_t_num : -1);

        bool hasM = obj.MS.fm[i] > -1;
        MS.fm.push_back(hasM ? obj.MS.fm[i] + MT_num : -1);
    }

    // Update counts
    MS.v_num = MS.vx.size();
    MS.n_num = MS.nx.size();
    MS.t_num = MS.tx.size();
    MS.f_num = MS.fv0.size();
    MS.l_num = MS.lsrc.size();

    MT.num = MT.Alb_r.size();
    TX.num = TX.w.size();
    TX.size = TX.r.size();
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
AzGlobal::~AzGlobal() {
    d_MS.free();
    d_MT.free();
    d_TX.free();

    delete[] min_x; delete[] min_y; delete[] min_z;
    delete[] max_x; delete[] max_y; delete[] max_z;
    delete[] fcx;   delete[] fcy;   delete[] fcz;
}

void AzGlobal::toDevice() {
    d_MS.copy(MS);
    d_MT.copy(MT);
    d_TX.copy(TX);
}

void AzGlobal::gulp(AzObj &obj) {
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
        MT.AlbMap.push_back(am > -1 ? am + TX_num : 0);
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

        bool hasN = obj.MS.fn0[i] > -1;
        MS.fn0.push_back(hasN ? obj.MS.fn0[i] + MS_n_num : 0);
        MS.fn1.push_back(hasN ? obj.MS.fn1[i] + MS_n_num : 0);
        MS.fn2.push_back(hasN ? obj.MS.fn2[i] + MS_n_num : 0);

        bool hasT = obj.MS.ft0[i] > -1;
        MS.ft0.push_back(hasT ? obj.MS.ft0[i] + MS_t_num : 0);
        MS.ft1.push_back(hasT ? obj.MS.ft1[i] + MS_t_num : 0);
        MS.ft2.push_back(hasT ? obj.MS.ft2[i] + MS_t_num : 0);

        bool hasM = obj.MS.fm[i] > -1;
        MS.fm.push_back(hasM ? obj.MS.fm[i] + MT_num : 0);
    }

    // Update counts
    MS.v_num = MS.vx.size();
    MS.n_num = MS.nx.size();
    MS.t_num = MS.tx.size();
    MS.f_num = MS.fv0.size();
    MS.l_num = MS.lsrc.size();

    MT.num = MT.Alb_r.size();
    TX.num = TX.w.size();
    TX.size = TX.r.size();
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
    min_x = new float[MS.f_num];
    min_y = new float[MS.f_num];
    min_z = new float[MS.f_num];
    max_x = new float[MS.f_num];
    max_y = new float[MS.f_num];
    max_z = new float[MS.f_num];
    fcx = new float[MS.f_num];
    fcy = new float[MS.f_num];
    fcz = new float[MS.f_num];

    cudaMemcpy(min_x, d_MS.min_x, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(min_y, d_MS.min_y, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(min_z, d_MS.min_z, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(max_x, d_MS.max_x, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(max_y, d_MS.max_y, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(max_z, d_MS.max_z, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(fcx, d_MS.fcx, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(fcy, d_MS.fcy, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(fcz, d_MS.fcz, MS.f_num * sizeof(float), cudaMemcpyDeviceToHost);
}