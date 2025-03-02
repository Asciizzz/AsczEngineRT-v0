#include <AsczBvh.cuh>
#include <ToDevice.cuh>
#include <algorithm>

void AsczBvh::freeDevice() {
    if (nNum == 0) return;

    cudaFree(d_mi_x);
    cudaFree(d_mi_y);
    cudaFree(d_mi_z);
    cudaFree(d_mx_x);
    cudaFree(d_mx_y);
    cudaFree(d_mx_z);
    cudaFree(d_cl);
    cudaFree(d_cr);
    cudaFree(d_ll);
    cudaFree(d_lr);
    cudaFree(d_gIdx);
}
void AsczBvh::toDevice() {
    freeDevice();

    nNum = h_mi_x.size();
    ToDevice::F(h_mi_x, d_mi_x, nNum);
    ToDevice::F(h_mi_y, d_mi_y, nNum);
    ToDevice::F(h_mi_z, d_mi_z, nNum);
    ToDevice::F(h_mx_x, d_mx_x, nNum);
    ToDevice::F(h_mx_y, d_mx_y, nNum);
    ToDevice::F(h_mx_z, d_mx_z, nNum);
    ToDevice::I(h_cl, d_cl, nNum);
    ToDevice::I(h_cr, d_cr, nNum);
    ToDevice::I(h_ll, d_ll, nNum);
    ToDevice::I(h_lr, d_lr, nNum);
    ToDevice::I(h_gIdx, d_gIdx);
}


int AsczBvh::buildBvh(
    VecF &mi_x, VecF &mi_y, VecF &mi_z, VecF &mx_x, VecF &mx_y, VecF &mx_z,
    VecI &cl, VecI &cr, VecI &ll, VecI &lr,

    VecI &allGIdx, DevNode &node, const VecAB &ABs,
    int depth, const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    // allNode.push_back(node);
    mi_x.push_back(node.ab.min.x);
    mi_y.push_back(node.ab.min.y);
    mi_z.push_back(node.ab.min.z);
    mx_x.push_back(node.ab.max.x);
    mx_y.push_back(node.ab.max.y);
    mx_z.push_back(node.ab.max.z);
    cl.push_back(node.cl);
    cr.push_back(node.cr);
    ll.push_back(node.ll);
    lr.push_back(node.lr);

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

    int curIdx = mi_x.size() - 1;

    // allNode[curIdx].cl = allNode.size();
    cl[curIdx] = mi_x.size();
    idx += buildBvh(
        mi_x, mi_y, mi_z, mx_x, mx_y, mx_z, cl, cr, ll, lr,
        allGIdx, l, ABs, depth + 1,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    cr[curIdx] = mi_x.size();
    idx += buildBvh(
        mi_x, mi_y, mi_z, mx_x, mx_y, mx_z, cl, cr, ll, lr,
        allGIdx, r, ABs, depth + 1,
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
        h_mi_x, h_mi_y, h_mi_z, h_mx_x, h_mx_y, h_mx_z,
        h_cl, h_cr, h_ll, h_lr,
        h_gIdx, root, G_AB, 0,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );
}