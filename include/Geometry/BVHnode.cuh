#ifndef HstNode_H
#define HstNode_H

#include <Geom.cuh>

struct HstNode {
    Vec3f min;
    Vec3f max;
    int l;
    int r;
    bool leaf;
};

#endif