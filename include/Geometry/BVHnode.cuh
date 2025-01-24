#ifndef BVHNODE_H
#define BVHNODE_H

#include <Geom.cuh>

struct BVHnode {
    Vec3f min;
    Vec3f max;
    int l;
    int r;
    bool leaf;
};

#endif