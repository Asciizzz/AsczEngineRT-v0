#ifndef ASCZBVH_CUH
#define ASCZBVH_CUH

#include <AsczMesh.cuh>

/* 3 Level of BVH construction

Lvl 1: Scenewise BVH, split objects
Lvl 2: Objectwise BVH, split sub-objects
Lvl 3: Sub-objectwise BVH, split faces

Refer to AsczMesh.cuh for detail regarding object and sub-object

*/

#define VecNode std::vector<DevNode>

struct DevNode { // Flattened structure friendly for shader code
    float min_x= INFINITY, min_y= INFINITY, min_z= INFINITY;
    float max_x=-INFINITY, max_y=-INFINITY, max_z=-INFINITY;

    int cl = -1, cr = -1; // Children
    int ll = -1, lr = -1; // Primitive

    int depth = 0;
};


class AsczBvh {
public:
    ~AsczBvh();

    std::vector<int> h_fIdx;
    int *d_fIdx = nullptr;

    VecNode h_nodes;
    DevNode *d_nodes = nullptr;

    float *d_min_x, *d_min_y, *d_min_z; // AABB Min
    float *d_max_x, *d_max_y, *d_max_z; // AABB Max
    int   *d_pl, *d_pr;     // Dual Purpose pointer
    bool  *d_lf;                       // Leaf flag

    int nNum = 0;

    int MAX_DEPTH = 32;
    int NODE_FACES = 1;
    int BIN_COUNT = 11;

    void toDevice();

    static int buildBvh(
        VecNode &nodes, std::vector<int> &fIdxs,
        const std::vector<float> &min_x, const std::vector<float> &min_y, const std::vector<float> &min_z,
        const std::vector<float> &max_x, const std::vector<float> &max_y, const std::vector<float> &max_z,
        const std::vector<float> &c_x, const std::vector<float> &c_y, const std::vector<float> &c_z,
        const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
    );


    void designBVH(AsczMesh &meshMgr);
};

#endif