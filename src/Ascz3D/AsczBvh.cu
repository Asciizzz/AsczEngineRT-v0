#include <AsczBvh.cuh>

#include <ToDevice.cuh>
#include <algorithm>
#include <execution>
#include <omp.h>

__global__ void toSoAKernel(
    float *mi_x, float *mi_y, float *mi_z,
    float *mx_x, float *mx_y, float *mx_z,
    int *pl, int *pr, bool *lf,
    DevNode *nodes, int nNum
) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= nNum) return;

    mi_x[idx] = nodes[idx].ab.min.x;
    mi_y[idx] = nodes[idx].ab.min.y;
    mi_z[idx] = nodes[idx].ab.min.z;

    mx_x[idx] = nodes[idx].ab.max.x;
    mx_y[idx] = nodes[idx].ab.max.y;
    mx_z[idx] = nodes[idx].ab.max.z;

    lf[idx] = nodes[idx].cl == -1;

    pl[idx] = lf[idx] ? nodes[idx].ll : nodes[idx].cl;
    pr[idx] = lf[idx] ? nodes[idx].lr : nodes[idx].cr;
}

AsczBvh::~AsczBvh() {
    cudaFree(d_nodes);
    cudaFree(d_mi_x);
    cudaFree(d_mi_y);
    cudaFree(d_mi_z);
    cudaFree(d_mx_x);
    cudaFree(d_mx_y);
    cudaFree(d_mx_z);
    cudaFree(d_pl);
    cudaFree(d_pr);
    cudaFree(d_lf);
    cudaFree(d_gIdx);
}

void AsczBvh::toDevice() {
    nNum = h_nodes.size();

    cudaMalloc(&d_nodes, nNum * sizeof(DevNode));
    cudaMalloc(&d_mi_x, nNum * sizeof(float));
    cudaMalloc(&d_mi_y, nNum * sizeof(float));
    cudaMalloc(&d_mi_z, nNum * sizeof(float));
    cudaMalloc(&d_mx_x, nNum * sizeof(float));
    cudaMalloc(&d_mx_y, nNum * sizeof(float));
    cudaMalloc(&d_mx_z, nNum * sizeof(float));
    cudaMalloc(&d_pl, nNum * sizeof(int));
    cudaMalloc(&d_pr, nNum * sizeof(int));
    cudaMalloc(&d_lf, nNum * sizeof(bool));

    cudaMemcpy(d_nodes, h_nodes.data(), nNum * sizeof(DevNode), cudaMemcpyHostToDevice);

    toSoAKernel<<<nNum / 256 + 1, 256>>>(
        d_mi_x, d_mi_y, d_mi_z,
        d_mx_x, d_mx_y, d_mx_z,
        d_pl, d_pr, d_lf,
        d_nodes, nNum
    );

    ToDevice::I(h_gIdx, d_gIdx);
}


int AsczBvh::buildBvh(
    VecNode &allNodes, std::vector<int> &allGIdx, DevNode &node, const std::vector<AABB> &ABs,
    int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    allNodes.push_back(node);

    int idx = 0;

    int nG = node.lr - node.ll;
    if (nG <= NODE_FACES || depth >= MAX_DEPTH) {
        node.cl = -1;
        node.cr = -1;
        return 1;
    }

    AABB nAB = node.ab;
    Flt3 nAB_ = nAB.max - nAB.min;
    float curCost = nAB.getSA() * nG;

    int bestAxis = -1;
    int bestSplit = -1;
    AABB bestLab, bestRab;
    float bestCost = curCost;

    for (int a = 0; a < 3; ++a) {
        std::sort(std::execution::par,
        allGIdx.begin() + node.ll, allGIdx.begin() + node.lr,
        [&](int i1, int i2) {
            return ABs[i1].cent()[a] < ABs[i2].cent()[a];
        });

        #pragma omp parallel
        for (int b = 0; b < BIN_COUNT; ++b) {
            DevNode l, r;

            float splitPoint = nAB.min[a] + nAB_[a] * (b + 1) / BIN_COUNT;

            int splitIdx = node.ll;

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

            float lCost = l.ab.getSA() * (splitIdx - node.ll);
            float rCost = r.ab.getSA() * (node.lr - splitIdx);
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

    std::sort(std::execution::par,
    allGIdx.begin() + node.ll, allGIdx.begin() + node.lr,
    [&](int i1, int i2) {
        return ABs[i1].cent()[bestAxis] < ABs[i2].cent()[bestAxis];
    });

    DevNode l = { bestLab, -1, -1, node.ll, bestSplit };
    DevNode r = { bestRab, -1, -1, bestSplit, node.lr };

    int curIdx = allNodes.size() - 1;

    allNodes[curIdx].cl = allNodes.size();
    idx += buildBvh(
        allNodes, allGIdx, l, ABs, depth + 1,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    allNodes[curIdx].cr = allNodes.size();
    idx += buildBvh(
        allNodes, allGIdx, r, ABs, depth + 1,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    return idx + 1;
}

void AsczBvh::designBVH(AsczMesh &meshMgr) {
    const int &gNum = meshMgr.gNum;
    const AABB &GlbAB = meshMgr.GlbAB;
    // const std::vector<AABB> &O_AB = meshMgr.O_AB;
    // const std::vector<AABB> &SO_AB = meshMgr.SO_AB;
    const std::vector<AABB> &G_AB = meshMgr.G_AB;

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