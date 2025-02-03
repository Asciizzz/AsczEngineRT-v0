#ifndef BVHMANAGER_CUH
#define BVHMANAGER_CUH

#include <MeshManager.cuh>

#include <string>

struct HstNode { // Host code node
    Vec3f min = Vec3f(INFINITY);
    Vec3f max = Vec3f(-INFINITY);

    HstNode *l = nullptr, *r = nullptr; // Child node

    bool leaf = false;
    std::vector<int> faces; // Face indices

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

struct DevNode { // Flattened structure friendly for shader code
    Vec3f min = Vec3f(INFINITY);
    Vec3f max = Vec3f(-INFINITY);

    int l, r; // Child node index
    bool leaf = false;

    int fl, fr; // [face left, face right)
};

class BvhManager {
public:
    std::vector<HstNode> h_nodes;

    int appendNode(HstNode node) {
        h_nodes.push_back(node);
        return num++;
    }

    HstNode *d_nodes;
    int num = 0;

    void hostToDevice() {
        cudaMalloc(&d_nodes, num * sizeof(HstNode));
        cudaMemcpy(d_nodes, h_nodes.data(), num * sizeof(HstNode), cudaMemcpyHostToDevice);
    }

    void designBVH(MeshManager &meshMgr) {
        const Vecs3i &fv = meshMgr.h_fv; // Faces

        const VecsI &OrSO = meshMgr.OrSO; // Object references sub-objects
        const VecsI &SOrF = meshMgr.SOrF; // Sub-object references faces

        const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
        const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
        const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center

        // For the time being, we will focus on treating each sub-objects as a leaf node
        // And try to perform SAH BVH on those sub-objects instead of their faces

        // We will perform the simplest BVH construction algorithm
        // Split in the middle of the longest axis

        HstNode *root = new HstNode();
        root->faces.resize(fv.size());
        for (int i = 0; i < fv.size(); i++) {
            root->faces[i] = i;
            root->recalc(fABmin[i]);
            root->recalc(fABmax[i]);
        }

        buildBvh(root, meshMgr);
    }

    static void buildBvh(HstNode *nodes, MeshManager &meshMgr, int depth=0, std::string pf="O ") {
        const VecsI &OrSO = meshMgr.OrSO; // Object references sub-objects
        const VecsI &SOrF = meshMgr.SOrF; // Sub-object references faces

        const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
        const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
        const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center

        const int MAX_DEPTH = 5;

        HstNode *left = new HstNode();
        HstNode *right = new HstNode();
        nodes->l = left;
        nodes->r = right;

        Vec3f AABBsize = nodes->max - nodes->min;
        int axis = 0;
        if (AABBsize.y > AABBsize.x) axis = 1;
        if (AABBsize.z > AABBsize.y) axis = 2;

        for (int i = 0; i < nodes->faces.size(); i++) {
            int idx = nodes->faces[i];
            Vec3f center = fABcen[idx];

            if (center[axis] < nodes->min[axis] + AABBsize[axis] / 2) {
                left->faces.push_back(idx);
                left->recalc(fABmin[idx]);
                left->recalc(fABmax[idx]);
            } else {
                right->faces.push_back(idx);
                right->recalc(fABmin[idx]);
                right->recalc(fABmax[idx]);
            }
        }

        int lF = left->faces.size();
        int rF = right->faces.size();
        int nF = nodes->faces.size();

        nodes->leaf = depth >= MAX_DEPTH || nF <= 1 || lF == nF || rF == nF;

        std::string leafStr = nodes->leaf ? "lf:[" : "nd:[";
        std::string axisStr = axis == 0 ? "x" : (axis == 1 ? "y" : "z");
        std::cout << depth << " - " << axisStr << " | " << pf << leafStr << nF << "] | ";
        Vec3f min = nodes->min, max = nodes->max;
        std::cout << "(" << min.x << ", " << min.y << ", " << min.z << ") | ";
        std::cout << "(" << max.x << ", " << max.y << ", " << max.z << ")\n";

        if (nodes->leaf) return;

        buildBvh(left, meshMgr, depth + 1, pf + "L ");
        buildBvh(right, meshMgr, depth + 1, pf + "R ");
    }
};

#endif