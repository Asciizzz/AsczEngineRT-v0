#ifndef STACKBVH_CUH
#define STACKBVH_CUH

#include <Geom.cuh>
#include <BVHNode.cuh>
#include <vector>

// A nested BVH structure (that will be flattened into a stackless BVH) 
class StackBVH {
public:
    Vec3f min;
    Vec3f max;
    std::vector<StackBVH> nodes;
    std::vector<Geom> leafs;

    StackBVH(std::vector<StackBVH> nodes) : nodes(nodes) {}
    StackBVH(std::vector<Geom> leafs) : leafs(leafs) {}
};

#endif