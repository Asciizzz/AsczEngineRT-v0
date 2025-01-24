#ifndef BVHMANAGER_CUH
#define BVHMANAGER_CUH

// Probably the most complicated part
// Surface area heuristic? More like Surface area autistic
// Fucking help me, it is currently 1:15 AM

#include <MeshManager.cuh>

struct NodeHst { // For host memory
    Vec3f ABmin, ABmax;

    NodeHst *left;
    NodeHst *right;

    bool leaf = false;
    std::vector<int> faces;
};

struct NodeDev { // For device memory
    Vec3f ABmin, ABmax;

    // Left and right serve dual purpose
    // If node, then it is the index of the node
    // If leaf, then it is the start and end index of the faces
    int left, right;
    bool leaf = false;
};

class BvhManager {
public:
    // Host memory
    NodeHst *h_nodes;
    std::vector<int> h_faceMap;

    // For the time being, we will split the BVH into 2
    // based on the longest axis
    static void buildBvh(
        NodeHst *node, Vec3f ABmin, Vec3f ABmax,
        Vecs3f mv, Vecs3i mfv, VectI mfi, int depth=0
    ) {
        if (mfi.size() == 0) return;

        if (mfi.size() < 32 || depth > 24) {
            node->leaf = true;
            node->faces = mfi;
            return;
        }

        Vec3f ABdif = ABmax - ABmin;

        int axis = 0; // 0 = x, 1 = y, 2 = z
        if (ABdif.y > ABdif.x) axis = 1;
        if (ABdif.z > ABdif.y) axis = 2;

        // Find the middle value along the axis
        float mid = (ABmin[axis] + ABmax[axis]) / 2;

        VectI leftFaces, rightFaces;
        // Intialize the AABB with INFINITY and -INFINITY
        Vec3f ABminL = Vec3f(INFINITY, INFINITY, INFINITY);
        Vec3f ABmaxL = Vec3f(-INFINITY, -INFINITY, -INFINITY);
        Vec3f ABminR = Vec3f(INFINITY, INFINITY, INFINITY);
        Vec3f ABmaxR = Vec3f(-INFINITY, -INFINITY, -INFINITY);

        for (int i = 0; i < mfi.size(); i++) {
            Vec3i fv = mfv[mfi[i]];

            Vec3f v0 = mv[fv.x];
            Vec3f v1 = mv[fv.y];
            Vec3f v2 = mv[fv.z];

            Vec3f vMin = Vec3f(
                std::min(v0.x, std::min(v1.x, v2.x)),
                std::min(v0.y, std::min(v1.y, v2.y)),
                std::min(v0.z, std::min(v1.z, v2.z))
            );

            Vec3f vMax = Vec3f(
                std::max(v0.x, std::max(v1.x, v2.x)),
                std::max(v0.y, std::max(v1.y, v2.y)),
                std::max(v0.z, std::max(v1.z, v2.z))
            );

            if (vMin[axis] < mid) {
                leftFaces.push_back(mfi[i]);
                ABminL = Vec3f(
                    std::min(ABminL.x, vMin.x),
                    std::min(ABminL.y, vMin.y),
                    std::min(ABminL.z, vMin.z)
                );
                ABmaxL = Vec3f(
                    std::max(ABmaxL.x, vMax.x),
                    std::max(ABmaxL.y, vMax.y),
                    std::max(ABmaxL.z, vMax.z)
                );
            } else {
                rightFaces.push_back(mfi[i]);
                ABminR = Vec3f(
                    std::min(ABminR.x, vMin.x),
                    std::min(ABminR.y, vMin.y),
                    std::min(ABminR.z, vMin.z)
                );
                ABmaxR = Vec3f(
                    std::max(ABmaxR.x, vMax.x),
                    std::max(ABmaxR.y, vMax.y),
                    std::max(ABmaxR.z, vMax.z)
                );
            }
        }

        std::string indent = "   ";

        std::cout << "Depth:" << depth << "Faces: " << mfi.size() << " - Left: " << leftFaces.size() << " - Right: " << rightFaces.size() << std::endl;
        std::cout << indent << "- ABmin: " << ABmin.x << " " << ABmin.y << " " << ABmin.z << std::endl;
        std::cout << indent << "- ABmax: " << ABmax.x << " " << ABmax.y << " " << ABmax.z << std::endl;
        if (axis == 0) std::cout << indent << "- Axis: x";
        if (axis == 1) std::cout << indent << "- Axis: y";
        if (axis == 2) std::cout << indent << "- Axis: z";
        std::cout << " - Mid: " << mid << std::endl;
        std::cout << indent << "- - ABminL: " << ABminL.x << " " << ABminL.y << " " << ABminL.z << std::endl;
        std::cout << indent << "- - ABmaxL: " << ABmaxL.x << " " << ABmaxL.y << " " << ABmaxL.z << std::endl;
        std::cout << indent << "- - ABminR: " << ABminR.x << " " << ABminR.y << " " << ABminR.z << std::endl;
        std::cout << indent << "- - ABmaxR: " << ABmaxR.x << " " << ABmaxR.y << " " << ABmaxR.z << std::endl;

        node->left = new NodeHst();
        node->right = new NodeHst();

        buildBvh(node->left, ABminL, ABmaxL, mv, mfv, leftFaces, depth + 1);
        buildBvh(node->right, ABminR, ABmaxR, mv, mfv, rightFaces, depth + 1);
    }
};

#endif