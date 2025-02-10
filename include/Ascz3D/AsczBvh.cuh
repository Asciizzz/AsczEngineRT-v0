#ifndef ASCZBVH_CUH
#define ASCZBVH_CUH

#include <AsczMesh.cuh>

#include <string>
#include <omp.h>

/* 3 Level of BVH construction

Lvl 1: Scenewise BVH, split objects
Lvl 2: Objectwise BVH, split sub-objects
Lvl 3: Sub-objectwise BVH, split faces

Keep in mind for the HstNode:

If its a leaf, then the faces vector is pretty much useless

*/

#define VecNode std::vector<DevNode>

struct DevNode { // Flattened structure friendly for shader code
    Flt3 min = Flt3(INFINITY);
    Flt3 max = Flt3(-INFINITY);

    int cl, cr; // Children
    int ll = -1;
    int lr = -1;

    _dev_ float hitDist(const Flt3 &rO, const Flt3 &rInvD) const;

    _hst_ void recalcMin(const Flt3 &v);
    _hst_ void recalcMax(const Flt3 &v);

    _hst_ static float findSAH(Flt3 min, Flt3 max) {
        Flt3 size = max - min;
        return size.x * size.y + size.y * size.z + size.z * size.x;
    }
    _hst_ static float findCost(Flt3 min, Flt3 max, int gNum) {
        return findSAH(min, max) * gNum;
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
    int buildBvh(
        
        DevNode &node, int depth);
    void designBVH(AsczMesh &meshMgr);
};

#endif