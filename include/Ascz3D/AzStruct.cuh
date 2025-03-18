#ifndef AZSTRUCT_CUH
#define AZSTRUCT_CUH

#include <vector>

#include <stb_image.h>

struct AzMesh {
    std::vector<float> vx, vy, vz;
    std::vector<float> nx, ny, nz;
    std::vector<float> tx, ty;

    std::vector<int> fv0, fv1, fv2;
    std::vector<int> fn0, fn1, fn2;
    std::vector<int> ft0, ft1, ft2;

    std::vector<int> fm;
    std::vector<int> lsrc;

    int v_num = 0, n_num = 0, t_num = 0, f_num = 0, l_num = 0;
};

struct D_AzMesh {
    float *vx, *vy, *vz;
    float *nx, *ny, *nz;
    float *tx, *ty;

    int *fv0, *fv1, *fv2;
    int *fn0, *fn1, *fn2;
    int *ft0, *ft1, *ft2;

    int *fm;
    int *lsrc;

    float *min_x, *min_y, *min_z;
    float *max_x, *max_y, *max_z;
    float *fcx, *fcy, *fcz;

    int v_num = 0, n_num = 0, t_num = 0, f_num = 0, l_num = 0;

    void copy(AzMesh &ms);
    void free();
};



struct AzMtl {
    std::vector<float> Alb_r, Alb_g, Alb_b;
    std::vector<int> AlbMap;

    std::vector<float> Rough, Metal;
    std::vector<float> Tr, Ior;

    std::vector<float> Ems_r, Ems_g, Ems_b, Ems_i;

    int num = 0;

    int push();
};

struct D_AzMtl {
    float *Alb_r, *Alb_g, *Alb_b;
    int *AlbMap;

    float *Rough, *Metal;
    float *Tr, *Ior;

    float *Ems_r, *Ems_g, *Ems_b, *Ems_i;

    int num = 0;

    void copy(AzMtl &mt);
    void free();
};


struct AzTxtr {
    std::vector<float> r, g, b, a;
    std::vector<int> w, h, off;

    int size = 0, num = 0;

    int append(const char *path);
};

struct D_AzTxtr {
    float *r, *g, *b, *a;
    int *w, *h, *off;

    int size = 0, num = 0;

    void copy(AzTxtr &tx);
    void free();
};

struct AzObj {
    AzMesh MS;
    AzMtl MT;
    AzTxtr TX;

    float AB_x =  INFINITY, AB_y =  INFINITY, AB_z =  INFINITY; // AABB min
    float AB_X = -INFINITY, AB_Y = -INFINITY, AB_Z = -INFINITY; // AABB max

    void updateNumbers() {
        MS.v_num = MS.vx.size();
        MS.n_num = MS.nx.size();
        MS.t_num = MS.tx.size();
        MS.f_num = MS.fv0.size();
        MS.l_num = MS.lsrc.size();

        MT.num = MT.Alb_r.size();
        TX.num = TX.w.size();
        TX.size = TX.r.size();
    }

    // Save/Load .azb file
    static void save(AzObj &Obj, const char *path);
    static AzObj load(const char *path);

    void combine(AzObj &obj);
};

struct AzGlobal {
    AzMesh MS;
    AzMtl MT;
    AzTxtr TX;

    D_AzMesh d_MS;
    D_AzMtl d_MT;
    D_AzTxtr d_TX;

    // Faces' AABB and faces' centroid
    float *min_x, *min_y, *min_z;
    float *max_x, *max_y, *max_z;
    float *fcx, *fcy, *fcz; // Not the AABB's centroid

    AzGlobal();
    ~AzGlobal();
    void gulp(AzObj &obj);

    void toDevice();
    void computeAB();
};

#endif