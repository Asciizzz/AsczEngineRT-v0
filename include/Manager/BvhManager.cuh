#ifndef BVHMANAGER_CUH
#define BVHMANAGER_CUH

#include <MeshManager.cuh>

struct BvhNode {
    Vec3f min = Vec3f(INFINITY);
    Vec3f max = Vec3f(-INFINITY);
    int fl, fr; // [face left, face right)

    void recalc(Vec3f v) {
        min.x = fminf(min.x, v.x);
        min.y = fminf(min.y, v.y);
        min.z = fminf(min.z, v.z);

        max.x = fmaxf(max.x, v.x);
        max.y = fmaxf(max.y, v.y);
        max.z = fmaxf(max.z, v.z);
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

    void bvh(VectI &h_fo, Vecs3i &h_fv, Vecs3f &h_fmin, Vecs3f &h_fmax) {
        for (int i = 0; i < h_fo.size() - 1; i++) {
            BvhNode node;
            node.fl = h_fo[i];
            node.fr = h_fo[i + 1];
            for (int j = h_fo[i]; j < h_fo[i + 1]; j++) {
                node.recalc(h_fmin[j]);
                node.recalc(h_fmax[j]);
            }
            appendNode(node);
        }
    }
};

#endif