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

struct HstNode { // Host code node
    Flt3 min = Flt3(INFINITY);
    Flt3 max = Flt3(-INFINITY);

    HstNode *l = nullptr, *r = nullptr; // Child node

    bool leaf = false;
    std::vector<int> geoms; // Geometry indices

    void recalcMin(Flt3 v);
    void recalcMax(Flt3 v);
    float findCost();
};

struct DevNode { // Flattened structure friendly for shader code
    Flt3 min = Flt3(INFINITY);
    Flt3 max = Flt3(-INFINITY);

    int l, r; // Dual purpose, either child node or face index
    bool leaf = false;

    _dev_ float hitDist(Flt3 rO, Flt3 rInvD) const;
};

class AsczBvh {
public:
    std::vector<HstNode> h_nodes;
    std::vector<DevNode> h_dnodes;
    VecI h_gIdx;

    Vec3f h_ABmin; // Geom's AABB min
    Vec3f h_ABmax; // Geom's AABB max
    Vec3f h_gCent; // Geom's centroid (not the AABB centroid)

    Flt3 *d_ABmin = nullptr;
    Flt3 *d_ABmax = nullptr;
    Flt3 *d_gCent = nullptr;

    void initAABB(AsczMesh &meshMgr);

    DevNode *d_nodes;
    int *d_gIdx;
    int nNum;

    int MAX_DEPTH = 48;
    int NODE_FACES = 2;

    int SPLIT_X = 5;
    int SPLIT_Y = 5;
    int SPLIT_Z = 5;

    void toDevice();

    // Sub-object split faces
    void buildBvh(HstNode *nodes, AsczMesh &meshMgr, int depth=0);
    static int toShader(HstNode *node, std::vector<DevNode> &dnodes, std::vector<int> &fidx);
    void designBVH(AsczMesh &meshMgr);
};

#endif