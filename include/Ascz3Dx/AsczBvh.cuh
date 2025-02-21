#ifndef ASCZBVH_CUH
#define ASCZBVH_CUH

#include <AsczMesh.cuh>

#include <string>
#include <omp.h>

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
    int ll = -1;
    int lr = -1;

    __device__ float hitDist(const Flt3 &rO, const Flt3 &rInvD) const;

    __host__ static float findCost(AABB &ab, int nG) {
        return ab.getSA() * nG;
    }
};



class AsczBvh {
public:
    VecI h_gIdx; // Geom's index
    int *d_gIdx = nullptr;

    VecNode h_nodes;
    DevNode *d_nodes;
    int nNum;

    int MAX_DEPTH = 32;
    int NODE_FACES = 1;
    int BIN_COUNT = 11;

    void freeDevice();
    void toDevice();

    // Sub-object split faces
    static int buildBvh(
        VecNode &allNode, VecI &allGIdx, DevNode &node, const VecAB &ABs,
        int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
    );
    void designBVH(AsczMesh &meshMgr);
};

#endif