#ifndef BVHMANAGER_CUH
#define BVHMANAGER_CUH

#include <MeshManager.cuh>

#include <string>

/* 3 Level of BVH construction

Lvl 1: Scenewise BVH, split objects
Lvl 2: Objectwise BVH, split sub-objects
Lvl 3: Sub-objectwise BVH, split faces

Keep in mind for the HstNode:

If its a leaf, then the faces vector is pretty much useless

*/

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
};

struct DevNode { // Flattened structure friendly for shader code
    Vec3f min = Vec3f(INFINITY);
    Vec3f max = Vec3f(-INFINITY);

    int l, r; // Dual purpose, either child node or face index
    bool leaf = false;

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
    std::vector<HstNode> h_nodes;
    std::vector<DevNode> h_dnodes;
    std::vector<int> h_fidx;

    DevNode *d_nodes;
    int *d_fidx;
    int nNum;

    void toDevice() {
        nNum = h_dnodes.size();

        cudaMalloc(&d_nodes, h_dnodes.size() * sizeof(DevNode));
        cudaMemcpy(d_nodes, h_dnodes.data(), h_dnodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

        cudaMalloc(&d_fidx, h_fidx.size() * sizeof(int));
        cudaMemcpy(d_fidx, h_fidx.data(), h_fidx.size() * sizeof(int), cudaMemcpyHostToDevice);
    }

    void designBVH(MeshManager &meshMgr) {
        const Vecs3i &fv = meshMgr.h_fv; // Faces

        // const VecsI &OrSO = meshMgr.OrSO; // Object references sub-objects
        const VecsI &SOrF = meshMgr.SOrF; // Sub-object references faces

        const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
        const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
        // const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center

        HstNode *root = new HstNode();
        root->faces.resize(fv.size());
        for (int i = 0; i < fv.size(); i++) {
            root->faces[i] = i;
            root->recalc(fABmin[i]);
            root->recalc(fABmax[i]);
        }

        buildLvl3Bvh(root, meshMgr);

        toShader(root, h_dnodes, h_fidx);

        // for (int i = 0; i < h_dnodes.size(); i++) {
        //     DevNode dnode = h_dnodes[i];

        //     std::cout << i 
        //     << " - " << (dnode.leaf ? "lf" : "nd") 
        //     << " | " << dnode.l << " | " << dnode.r << " | "
        //     << "(" << dnode.min.x << ", " << dnode.min.y << ", " << dnode.min.z << ") | "
        //     << "(" << dnode.max.x << ", " << dnode.max.y << ", " << dnode.max.z << ")\n";
        // }
    }

    // Object split sub-objects
    static void buildLvl2Bvh(HstNode *nodes, MeshManager &meshMgr, int depth=0, std::string pf="O ") {
        const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
        const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
        const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center
        
        const VecsI &SOrF = meshMgr.SOrF; // Sub-object references faces
    }

    // Sub-object split faces
    static void buildLvl3Bvh(HstNode *nodes, MeshManager &meshMgr, int depth=0, std::string pf="O ") {
        const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
        const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
        const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center

        const int MAX_DEPTH = 16;


        HstNode *left = new HstNode();
        HstNode *right = new HstNode();
        nodes->l = left;
        nodes->r = right;

        int nF = nodes->faces.size();

        Vec3f AABBsize = nodes->max - nodes->min;
        int axis = 0;
        axis = AABBsize.y > AABBsize.x ? 1 : axis;
        axis = AABBsize.z > AABBsize.y ? 2 : axis;

        for (int i = 0; i < nF; i++) {
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

        nodes->leaf = depth >= MAX_DEPTH || nF <= 1 || lF == nF || rF == nF;

        std::string leafStr = nodes->leaf ? "lf:[" : "nd:[";
        std::string axisStr = axis == 0 ? "x" : (axis == 1 ? "y" : "z");
        std::cout << depth << " - " << axisStr << " | " << pf << leafStr << nF << "] | ";
        Vec3f min = nodes->min, max = nodes->max;
        std::cout << "(" << min.x << ", " << min.y << ", " << min.z << ") | ";
        std::cout << "(" << max.x << ", " << max.y << ", " << max.z << ")\n";

        if (nodes->leaf) return;

        buildLvl3Bvh(left, meshMgr, depth + 1, pf + "L ");
        buildLvl3Bvh(right, meshMgr, depth + 1, pf + "R ");
    }

    static int toShader(HstNode *node, std::vector<DevNode> &dnodes, std::vector<int> &fidx) {
        int idx = dnodes.size();
        dnodes.push_back(DevNode());

        dnodes[idx].min = node->min;
        dnodes[idx].max = node->max;

        if (node->leaf) {
            dnodes[idx].leaf = true;

            dnodes[idx].l = fidx.size();

            for (int i = 0; i < node->faces.size(); i++) {
                fidx.push_back(node->faces[i]);
            }

            dnodes[idx].r = fidx.size();

            return idx;
        }

        dnodes[idx].l = toShader(node->l, dnodes, fidx);
        dnodes[idx].r = toShader(node->r, dnodes, fidx);

        return idx;
    }
};

#endif