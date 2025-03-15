#include <AsczBvh.cuh>

#include <ToDevice.cuh>
#include <algorithm>
#include <execution>
#include <omp.h>

__global__ void toSoAKernel(
    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *pl, int *pr, bool *lf,
    DevNode *nodes, int nNum
) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= nNum) return;

    BV_min_x[idx] = nodes[idx].min_x;
    BV_min_y[idx] = nodes[idx].min_y;
    BV_min_z[idx] = nodes[idx].min_z;

    BV_max_x[idx] = nodes[idx].max_x;
    BV_max_y[idx] = nodes[idx].max_y;
    BV_max_z[idx] = nodes[idx].max_z;

    lf[idx] = nodes[idx].cl == -1;

    pl[idx] = lf[idx] ? nodes[idx].ll : nodes[idx].cl;
    pr[idx] = lf[idx] ? nodes[idx].lr : nodes[idx].cr;
}

AsczBvh::~AsczBvh() {
    cudaFree(d_nodes);
    cudaFree(d_min_x);
    cudaFree(d_min_y);
    cudaFree(d_min_z);
    cudaFree(d_max_x);
    cudaFree(d_max_y);
    cudaFree(d_max_z);
    cudaFree(d_pl);
    cudaFree(d_pr);
    cudaFree(d_lf);
    cudaFree(d_fIdx);
}

void AsczBvh::toDevice() {
    nNum = h_nodes.size();

    cudaMalloc(&d_nodes, nNum * sizeof(DevNode));
    cudaMalloc(&d_min_x, nNum * sizeof(float));
    cudaMalloc(&d_min_y, nNum * sizeof(float));
    cudaMalloc(&d_min_z, nNum * sizeof(float));
    cudaMalloc(&d_max_x, nNum * sizeof(float));
    cudaMalloc(&d_max_y, nNum * sizeof(float));
    cudaMalloc(&d_max_z, nNum * sizeof(float));
    cudaMalloc(&d_pl, nNum * sizeof(int));
    cudaMalloc(&d_pr, nNum * sizeof(int));
    cudaMalloc(&d_lf, nNum * sizeof(bool));

    cudaMemcpy(d_nodes, h_nodes.data(), nNum * sizeof(DevNode), cudaMemcpyHostToDevice);

    toSoAKernel<<<nNum / 256 + 1, 256>>>(
        d_min_x, d_min_y, d_min_z,
        d_max_x, d_max_y, d_max_z,
        d_pl, d_pr, d_lf,
        d_nodes, nNum
    );

    ToDevice::I(h_fIdx, d_fIdx);
}

void AsczBvh::designBVH(AsczMesh &MS) {
    // Initialize h_fIdx
    h_fIdx.resize(MS.fNum);
    #pragma omp parallel
    for (int i = 0; i < MS.fNum; ++i) h_fIdx[i] = i;

    h_nodes.push_back({
        MS.GLB_min_x, MS.GLB_min_y, MS.GLB_min_z,
        MS.GLB_max_x, MS.GLB_max_y, MS.GLB_max_z,
        -1, -1, 0, MS.fNum, 0
    });

    buildBvh(
        h_nodes, h_fIdx,
        MS.AB_min_x, MS.AB_min_y, MS.AB_min_z,
        MS.AB_max_x, MS.AB_max_y, MS.AB_max_z,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );
}



int AsczBvh::buildBvh(
    VecNode &nodes, std::vector<int> &fIdxs,
    const std::vector<float> &min_x, const std::vector<float> &min_y, const std::vector<float> &min_z,
    const std::vector<float> &max_x, const std::vector<float> &max_y, const std::vector<float> &max_z,
    const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    std::queue<int> queue;
    queue.push(0);

    while (!queue.empty()) {
        int nIdx = queue.front();
        queue.pop();

        DevNode nd = nodes[nIdx];

        int nF = nd.lr - nd.ll;
        if (nF <= NODE_FACES || nd.depth >= MAX_DEPTH) {
            nodes[nIdx].cl = -1;
            nodes[nIdx].cr = -1;
            continue;
        }

        float nLn_x = nd.max_x - nd.min_x;
        float nLn_y = nd.max_y - nd.min_y;
        float nLn_z = nd.max_z - nd.min_z;

        int bestAxis = -1;
        int bestSplit = -1;

        float bestLab_min_x, bestLab_min_y, bestLab_min_z;
        float bestLab_max_x, bestLab_max_y, bestLab_max_z;

        float bestRab_min_x, bestRab_min_y, bestRab_min_z;
        float bestRab_max_x, bestRab_max_y, bestRab_max_z;

        float bestCost = (nLn_x * nLn_x + nLn_y * nLn_y + nLn_z * nLn_z) * nF;

        for (int a = 0; a < 3; ++a) {
            bool ax = a == 0;
            bool ay = a == 1;
            bool az = a == 2;

            std::sort(std::execution::par,
            fIdxs.begin() + nd.ll, fIdxs.begin() + nd.lr,
            [&](int i1, int i2) {
                float c1 =  (min_x[i1] + max_x[i1]) * ax +
                            (min_y[i1] + max_y[i1]) * ay +
                            (min_z[i1] + max_z[i1]) * az;
                float c2 =  (min_x[i2] + max_x[i2]) * ax +
                            (min_y[i2] + max_y[i2]) * ay +
                            (min_z[i2] + max_z[i2]) * az;

                return c1 < c2;
            });

            for (int b = 0; b < BIN_COUNT; ++b) {
                DevNode cl, cr;

                float s1 = nd.min_x * ax + nd.min_y * ay + nd.min_z * az;
                float s2 = nLn_x * ax + nLn_y * ay + nLn_z * az;
                float splitPoint = s1 + s2 * (b + 1) / BIN_COUNT;

                int splitIdx = nd.ll;

                for (int g = nd.ll; g < nd.lr; ++g) {
                    int i = fIdxs[g];

                    float cent = (min_x[i] + max_x[i]) * ax + (min_y[i] + max_y[i]) * ay + (min_z[i] + max_z[i]) * az;
                    cent *= 0.5f;

                    if (cent < splitPoint) {
                        cl.min_x = fminf(cl.min_x, min_x[i]);
                        cl.min_y = fminf(cl.min_y, min_y[i]);
                        cl.min_z = fminf(cl.min_z, min_z[i]);

                        cl.max_x = fmaxf(cl.max_x, max_x[i]);
                        cl.max_y = fmaxf(cl.max_y, max_y[i]);
                        cl.max_z = fmaxf(cl.max_z, max_z[i]);

                        splitIdx++;
                    }
                    else {
                        cr.min_x = fminf(cr.min_x, min_x[i]);
                        cr.min_y = fminf(cr.min_y, min_y[i]);
                        cr.min_z = fminf(cr.min_z, min_z[i]);

                        cr.max_x = fmaxf(cr.max_x, max_x[i]);
                        cr.max_y = fmaxf(cr.max_y, max_y[i]);
                        cr.max_z = fmaxf(cr.max_z, max_z[i]);
                    }
                }

                float lCost = (cl.max_x - cl.min_x) * (cl.max_y - cl.min_y) * (cl.max_z - cl.min_z) * (splitIdx - nd.ll);
                float rCost = (cr.max_x - cr.min_x) * (cr.max_y - cr.min_y) * (cr.max_z - cr.min_z) * (nd.lr - splitIdx);
                float cost = lCost + rCost;

                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = a;
                    bestSplit = splitIdx;

                    bestLab_min_x = cl.min_x;
                    bestLab_min_y = cl.min_y;
                    bestLab_min_z = cl.min_z;

                    bestLab_max_x = cl.max_x;
                    bestLab_max_y = cl.max_y;
                    bestLab_max_z = cl.max_z;

                    bestRab_min_x = cr.min_x;
                    bestRab_min_y = cr.min_y;
                    bestRab_min_z = cr.min_z;

                    bestRab_max_x = cr.max_x;
                    bestRab_max_y = cr.max_y;
                    bestRab_max_z = cr.max_z;
                }
            }
        }

        if (bestAxis == -1) {
            nodes[nIdx].cl = -1;
            nodes[nIdx].cr = -1;
            continue;
        }

        std::sort(std::execution::par,
        fIdxs.begin() + nd.ll, fIdxs.begin() + nd.lr,
        [&](int i1, int i2) {
            float c1 =  (min_x[i1] + max_x[i1]) * (bestAxis == 0) +
                        (min_y[i1] + max_y[i1]) * (bestAxis == 1) +
                        (min_z[i1] + max_z[i1]) * (bestAxis == 2);

            float c2 =  (min_x[i2] + max_x[i2]) * (bestAxis == 0) +
                        (min_y[i2] + max_y[i2]) * (bestAxis == 1) +
                        (min_z[i2] + max_z[i2]) * (bestAxis == 2);

            return c1 < c2;
        });

        // Create left and right node
        DevNode nl = {
            bestLab_min_x, bestLab_min_y, bestLab_min_z,
            bestLab_max_x, bestLab_max_y, bestLab_max_z,
            -1, -1, nd.ll, bestSplit, nd.depth + 1
        };

        DevNode nr = {
            bestRab_min_x, bestRab_min_y, bestRab_min_z,
            bestRab_max_x, bestRab_max_y, bestRab_max_z,
            -1, -1, bestSplit, nd.lr, nd.depth + 1
        };

        int lIdx = nodes.size();
        nodes.push_back(nl);

        int rIdx = nodes.size();
        nodes.push_back(nr);

        nodes[nIdx].cl = lIdx;
        nodes[nIdx].cr = rIdx;

        queue.push(lIdx);
        queue.push(rIdx);
    }

    return 0;
}