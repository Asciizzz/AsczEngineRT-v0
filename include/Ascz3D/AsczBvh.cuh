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

    _dev_ float hitDist(const Flt3 &rO, const Flt3 &rInvD) const;

    _hst_ static float findCost(AABB &ab, int nG) {
        return ab.getSA() * nG;
    }
};



class AsczBvh {
public:

    Vec3f h_ABmin; // Geom's AABB min
    Vec3f h_ABmax; // Geom's AABB max
    Vec3f h_gCent; // Geom's centroid (not the AABB centroid)
    VecI h_gIdx; // Geom's index

    Flt3 *d_ABmin = nullptr;
    Flt3 *d_ABmax = nullptr;
    Flt3 *d_gCent = nullptr;
    int *d_gIdx = nullptr;

    void initAABB(AsczMesh &meshMgr);

    VecNode h_nodes;
    DevNode *d_nodes;
    int nNum;

    int MAX_DEPTH = 48;
    int NODE_FACES = 2;
    int BIN_COUNT = 5;

    void toDevice();

    // Sub-object split faces
    static int buildBvh(
        VecNode &allNode, VecI &allGIdx, DevNode &node, const AABB &ABs,
        int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
    );
    void designBVH(AsczMesh &meshMgr);
};

#endif