#ifndef BVHMANAGER_CUH
#define BVHMANAGER_CUH

#include <MeshManager.cuh>

struct BvhNode {
    Vec3f min = Vec3f(INFINITY);
    Vec3f max = Vec3f(-INFINITY);
    int fl, fr; // [fl, fr)

    void recalc(Vec3f v) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        min.z = std::min(min.z, v.z);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
        max.z = std::max(max.z, v.z);
    }

    // Ray hit AABB
    __device__
    bool hitAABB(Vec3f rO, Vec3f rD) const {
        float tmin = (min.x - rO.x) / rD.x;
        float tmax = (max.x - rO.x) / rD.x;

        if (tmin > tmax) {
            float temp = tmin;
            tmin = tmax;
            tmax = temp;
        }

        float tymin = (min.y - rO.y) / rD.y;
        float tymax = (max.y - rO.y) / rD.y;

        if (tymin > tymax) {
            float temp = tymin;
            tymin = tymax;
            tymax = temp;
        }
        
        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (min.z - rO.z) / rD.z;
        float tzmax = (max.z - rO.z) / rD.z;
        
        if (tzmin > tzmax) {
            float temp = tzmin;
            tzmin = tzmax;
            tzmax = temp;
        }

        return (tmin < tzmax && tzmin < tmax);
    }
};

class BvhManager {
public:
    std::vector<BvhNode> h_nodes;

    int appendNode(BvhNode node) {
        h_nodes.push_back(node);
        return num++;
    }

    BvhNode *d_nodes;
    int num = 0;

    void hostToDevice() {
        cudaMalloc(&d_nodes, num * sizeof(BvhNode));
        cudaMemcpy(d_nodes, h_nodes.data(), num * sizeof(BvhNode), cudaMemcpyHostToDevice);
    }
};

#endif