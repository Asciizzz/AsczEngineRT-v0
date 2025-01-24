#ifndef MESHMANAGER_CUH
#define MESHMANAGER_CUH

#include <Vector.cuh>

struct MeshStruct {
    Vecs3f v;
    Vecs2f t;
    Vecs3f n;

    Vecs3i fv;
    Vecs3i ft;
    Vecs3i fn;
    Vecs3i fm;
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
    Vecs3i h_fm;

    void appendMesh(MeshStruct mesh);

    // Device memory
    Vec3f *d_v = nullptr; int vNum = 0;
    Vec2f *d_t = nullptr; int tNum = 0;
    Vec3f *d_n = nullptr; int nNum = 0;

    int *d_fv = nullptr;
    int *d_ft = nullptr;
    int *d_fn = nullptr;
    int *d_fm = nullptr;
    int fNum = 0;

    void freeDevice();
    void hostToDevice();
};

#endif