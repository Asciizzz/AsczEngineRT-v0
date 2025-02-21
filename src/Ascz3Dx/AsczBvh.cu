#include <AsczBvh.cuh>

#include <algorithm>

float DevNode::hitDist(const Flt3 &rO, const Flt3 &rInvD) const {
    const Flt3 &min = ab.min;
    const Flt3 &max = ab.max;

    if (rO.x >= min.x && rO.x <= max.x &&
        rO.y >= min.y && rO.y <= max.y &&
        rO.z >= min.z && rO.z <= max.z) return 0.0f;

    float t1 = (min.x - rO.x) * rInvD.x;
    float t2 = (max.x - rO.x) * rInvD.x;
    float t3 = (min.y - rO.y) * rInvD.y;
    float t4 = (max.y - rO.y) * rInvD.y;
    float t5 = (min.z - rO.z) * rInvD.z;
    float t6 = (max.z - rO.z) * rInvD.z;

    float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
    float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

    if (tmax < tmin) return -1.0f; // No intersection
    if (tmin < 0.0f) return -1.0f; // Intersection behind the ray
    return tmin;
}



void AsczBvh::freeDevice() {
    if (nNum == 0) return;
    cudaFree(d_nodes);
    cudaFree(d_gIdx);
}
void AsczBvh::toDevice() {
    freeDevice();

    nNum = h_nodes.size();

    cudaMalloc(&d_nodes, h_nodes.size() * sizeof(DevNode));
    cudaMemcpy(d_nodes, h_nodes.data(), h_nodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

    cudaMalloc(&d_gIdx, h_gIdx.size() * sizeof(int));
    cudaMemcpy(d_gIdx, h_gIdx.data(), h_gIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
}


int AsczBvh::buildBvh(
    VecNode &allNode, VecI &allGIdx, DevNode &node, const VecAB &ABs,
    int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    allNode.push_back(node);

    int idx = 0;

    int nG = node.lr - node.ll;
    if (nG <= NODE_FACES || depth >= MAX_DEPTH) {
        node.cl = -1;
        node.cr = -1;
        return 1;
    }

    AABB nAB = node.ab;
    Flt3 nAB_ = nAB.max - nAB.min;
    float curCost = DevNode::findCost(node.ab, nG);

    int bestAxis = -1;
    int bestSplit = -1;
    AABB bestLab, bestRab;
    float bestCost = curCost;

    for (int a = 0; a < 3; ++a) {
        std::sort(allGIdx.begin() + node.ll, allGIdx.begin() + node.lr, [&](int i1, int i2) {
            return ABs[i1].cent()[a] < ABs[i2].cent()[a];
        });

        for (int b = 0; b < BIN_COUNT; ++b) {
            DevNode l, r;

            float splitPoint = nAB.min[a] + nAB_[a] * (b + 1) / BIN_COUNT;

            int splitIdx = node.ll;

            #pragma omp parallel
            for (int g = node.ll; g < node.lr; ++g) {
                int i = allGIdx[g];

                float cent = ABs[i].cent()[a];

                if (cent < splitPoint) {
                    l.ab.expandMin(ABs[i].min);
                    l.ab.expandMax(ABs[i].max);
                    splitIdx++;
                }
                else {
                    r.ab.expandMin(ABs[i].min);
                    r.ab.expandMax(ABs[i].max);
                }
            }

            float lCost = DevNode::findCost(l.ab, splitIdx - node.ll);
            float rCost = DevNode::findCost(r.ab, node.lr - splitIdx);
            float cost = lCost + rCost;

            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = a;
                bestSplit = splitIdx;

                bestLab = l.ab;
                bestRab = r.ab;
            }
        }
    }

    if (bestSplit == -1 || bestAxis == -1) {
        node.cl = -1;
        node.cr = -1;
        return 1;
    }

    std::sort(allGIdx.begin() + node.ll, allGIdx.begin() + node.lr, [&](int i1, int i2) {
        return ABs[i1].cent()[bestAxis] < ABs[i2].cent()[bestAxis];
    });

    DevNode l = { bestLab, -1, -1, node.ll, bestSplit };
    DevNode r = { bestRab, -1, -1, bestSplit, node.lr };

    int curIdx = allNode.size() - 1;

    allNode[curIdx].cl = allNode.size();
    idx += buildBvh(
        allNode, allGIdx, l, ABs, depth + 1,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    allNode[curIdx].cr = allNode.size();
    idx += buildBvh(
        allNode, allGIdx, r, ABs, depth + 1,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    return idx + 1;
}

void AsczBvh::designBVH(AsczMesh &meshMgr) {
    const int &gNum = meshMgr.gNum;
    const AABB &GlbAB = meshMgr.GlbAB;
    // const VecAB &O_AB = meshMgr.O_AB;
    // const VecAB &SO_AB = meshMgr.SO_AB;
    const VecAB &G_AB = meshMgr.G_AB;

    DevNode root = { GlbAB, -1, -1, 0, gNum };

    // Initialize h_gIdx
    h_gIdx.resize(gNum);
    #pragma omp parallel
    for (int i = 0; i < gNum; ++i) h_gIdx[i] = i;

    buildBvh(
        h_nodes, h_gIdx, root, G_AB, 0,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );
}