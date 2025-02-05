#ifndef HstNode_H
#define HstNode_H

#include <Geom.cuh>

struct HstNode {
    Flt3 min;
    Flt3 max;
    int l;
    int r;
    bool leaf;
};

#endif