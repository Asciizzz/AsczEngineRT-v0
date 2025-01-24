#ifndef MESHMANAGER_CUH
#define MESHMANAGER_CUH

#include <Vector.cuh>
#define VectI std::vector<int>

struct MeshStruct {
    Vecs3f v;
    Vecs2f t;
    Vecs3f n;

    Vecs3i fv;
    Vecs3i ft;
    Vecs3i fn;
    VectI fm;
};

class MeshManager {
public:
    // Host memory
    Vecs3f h_v;
    Vecs2f h_t;
    Vecs3f h_n;

    Vecs3i h_fv;
    Vecs3i h_ft;
    Vecs3i h_fn;
    VectI h_fm;

    void appendMesh(MeshStruct mesh);

    // Device memory
    Vec3f *d_v = nullptr; int vNum = 0;
    Vec2f *d_t = nullptr; int tNum = 0;
    Vec3f *d_n = nullptr; int nNum = 0;

    Vec3i *d_fv = nullptr;
    Vec3i *d_ft = nullptr;
    Vec3i *d_fn = nullptr;
    int *d_fm = nullptr;
    int fNum = 0;

    void freeDevice();
    void hostToDevice();
};

#endif