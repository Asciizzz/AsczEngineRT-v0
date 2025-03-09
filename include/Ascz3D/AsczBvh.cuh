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
    AABB ab; // AABB

    int cl, cr; // Children
    int ll, lr; // Primitive range (not necessarily faces)
};



class AsczBvh {
public:
    std::vector<int> h_gIdx; // Geom's index
    int *d_gIdx = nullptr;

    VecNode h_nodes;
    DevNode *d_nodes = nullptr;

    float *d_mi_x, *d_mi_y, *d_mi_z; // AABB Min
    float *d_mx_x, *d_mx_y, *d_mx_z; // AABB Max
    int *d_pl, *d_pr; // Dual Purpose pointer
    bool *d_lf; // Leaf flag

    int nNum;

    int MAX_DEPTH = 32;
    int NODE_FACES = 1;
    int BIN_COUNT = 11;

    void freeDevice();
    void toDevice();

    // Sub-object split faces
    static int buildBvh(
        // std::vector<float> &mi_x, std::vector<float> &mi_y, std::vector<float> &mi_z, std::vector<float> &mx_x, std::vector<float> &mx_y, std::vector<float> &mx_z,
        // std::vector<int> &cl, std::vector<int> &cr, std::vector<int> &ll, std::vector<int> &lr,
        VecNode &allNodes, std::vector<int> &allGIdx, DevNode &node, const VecAB &ABs,
        int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
    );
    void designBVH(AsczMesh &meshMgr);
};

#endif