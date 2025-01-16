#ifndef BVHNODE_CUH
#define BVHNODE_CUH

#include <Vector.cuh>

class BVHNode {
public:
    // AABB
    Vec3f min;
    Vec3f max;
    // Stackless Skip
    int skip;
    // For leaf
    bool isLeaf;
    int left = -1;
    int right = -1;
};

#endif