#include <AsczBvh.cuh>

#include <ToDevice.cuh>
#include <execution>
#include <omp.h>
#include <queue>

__global__ void toSoAKernel(
    float *BV_min_x, float *BV_min_y, float *BV_min_z,
    float *BV_max_x, float *BV_max_y, float *BV_max_z,
    int *pl, int *pr, bool *lf,
    AzNode *nodes, int nNum
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

    cudaMalloc(&d_nodes, nNum * sizeof(AzNode));
    cudaMalloc(&d_min_x, nNum * sizeof(float));
    cudaMalloc(&d_min_y, nNum * sizeof(float));
    cudaMalloc(&d_min_z, nNum * sizeof(float));
    cudaMalloc(&d_max_x, nNum * sizeof(float));
    cudaMalloc(&d_max_y, nNum * sizeof(float));
    cudaMalloc(&d_max_z, nNum * sizeof(float));
    cudaMalloc(&d_pl, nNum * sizeof(int));
    cudaMalloc(&d_pr, nNum * sizeof(int));
    cudaMalloc(&d_lf, nNum * sizeof(bool));

    cudaMemcpy(d_nodes, h_nodes.data(), nNum * sizeof(AzNode), cudaMemcpyHostToDevice);

    toSoAKernel<<<nNum / 256 + 1, 256>>>(
        d_min_x, d_min_y, d_min_z,
        d_max_x, d_max_y, d_max_z,
        d_pl, d_pr, d_lf,
        d_nodes, nNum
    );

    ToDevice::I(h_fIdx, d_fIdx);
}

void AsczBvh::designBVH(AzGlobal &Glb) {
    AzNode root = {
        0, 0, 0, 0, 0, 0,
        -1, -1, 0, Glb.MS.f_num, 0
    };

    h_fIdx.resize(Glb.MS.f_num);
    #pragma omp parallel
    for (int i = 0; i < Glb.MS.f_num; ++i) {
        h_fIdx[i] = i;

        float min_x = Glb.min_x[i], min_y = Glb.min_y[i], min_z = Glb.min_z[i];
        float max_x = Glb.max_x[i], max_y = Glb.max_y[i], max_z = Glb.max_z[i];

        root.min_x = fminf(root.min_x, min_x);
        root.min_y = fminf(root.min_y, min_y);
        root.min_z = fminf(root.min_z, min_z);

        root.max_x = fmaxf(root.max_x, max_x);
        root.max_y = fmaxf(root.max_y, max_y);
        root.max_z = fmaxf(root.max_z, max_z);
    }

    h_nodes.push_back(root);

    build_q(
        h_nodes, h_fIdx,
        Glb.min_x, Glb.min_y, Glb.min_z,
        Glb.max_x, Glb.max_y, Glb.max_z,
        Glb.fcx, Glb.fcy, Glb.fcz,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );
}



int AsczBvh::build_q(
    std::vector<AzNode> &nodes, std::vector<int> &fIdxs,
    const float *min_x, const float *min_y, const float *min_z,
    const float *max_x, const float *max_y, const float *max_z,
    const float *c_x, const float *c_y, const float *c_z,
    const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    std::queue<int> queue;
    queue.push(0);

    while (!queue.empty()) {
        int nIdx = queue.front();
        queue.pop();

        AzNode &nd = nodes[nIdx];

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

        #pragma omp parallel
        for (int i = 0; i < (BIN_COUNT - 1) * 3; ++i) {
            float lmin_x =  INFINITY, lmin_y =  INFINITY, lmin_z =  INFINITY;
            float lmax_x = -INFINITY, lmax_y = -INFINITY, lmax_z = -INFINITY;

            float rmin_x =  INFINITY, rmin_y =  INFINITY, rmin_z =  INFINITY;
            float rmax_x = -INFINITY, rmax_y = -INFINITY, rmax_z = -INFINITY;

            bool ax = i % 3 == 0, ay = i % 3 == 1, az = i % 3 == 2;
            int a = i % 3;
            int b = i / 3;

            float s1 = nd.min_x * ax + nd.min_y * ay + nd.min_z * az;
            float s2 = nLn_x    * ax + nLn_y    * ay + nLn_z    * az;
            float splitPoint = s1 + s2 * (b + 1) / BIN_COUNT;

            int splitIdx = nd.ll;

            for (int g = nd.ll; g < nd.lr; ++g) {
                int i = fIdxs[g];

                float cent = c_x[i] * ax + c_y[i] * ay + c_z[i] * az;

                if (cent < splitPoint) {
                    lmin_x = lmin_x < min_x[i] ? lmin_x : min_x[i];
                    lmin_y = lmin_y < min_y[i] ? lmin_y : min_y[i];
                    lmin_z = lmin_z < min_z[i] ? lmin_z : min_z[i];

                    lmax_x = lmax_x > max_x[i] ? lmax_x : max_x[i];
                    lmax_y = lmax_y > max_y[i] ? lmax_y : max_y[i];
                    lmax_z = lmax_z > max_z[i] ? lmax_z : max_z[i];

                    splitIdx++;
                }
                else {
                    rmin_x = rmin_x < min_x[i] ? rmin_x : min_x[i];
                    rmin_y = rmin_y < min_y[i] ? rmin_y : min_y[i];
                    rmin_z = rmin_z < min_z[i] ? rmin_z : min_z[i];

                    rmax_x = rmax_x > max_x[i] ? rmax_x : max_x[i];
                    rmax_y = rmax_y > max_y[i] ? rmax_y : max_y[i];
                    rmax_z = rmax_z > max_z[i] ? rmax_z : max_z[i];
                }
            }

            float lN_x = lmax_x - lmin_x, rN_x = rmax_x - rmin_x;
            float lN_y = lmax_y - lmin_y, rN_y = rmax_y - rmin_y;
            float lN_z = lmax_z - lmin_z, rN_z = rmax_z - rmin_z;

            float lCost = (lN_x * lN_x + lN_y * lN_y + lN_z * lN_z) * (splitIdx - nd.ll);
            float rCost = (rN_x * rN_x + rN_y * rN_y + rN_z * rN_z) * (nd.lr - splitIdx);

            float cost = lCost + rCost;

            #pragma omp critical
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = a;
                bestSplit = splitIdx;

                bestLab_min_x = lmin_x; bestRab_min_x = rmin_x;
                bestLab_min_y = lmin_y; bestRab_min_y = rmin_y;
                bestLab_min_z = lmin_z; bestRab_min_z = rmin_z;

                bestLab_max_x = lmax_x; bestRab_max_x = rmax_x;
                bestLab_max_y = lmax_y; bestRab_max_y = rmax_y;
                bestLab_max_z = lmax_z; bestRab_max_z = rmax_z;
            }
        }

        if (bestAxis == -1) {
            nodes[nIdx].cl = -1;
            nodes[nIdx].cr = -1;
            continue;
        }

        std::sort(fIdxs.begin() + nd.ll, fIdxs.begin() + nd.lr,
        [&](int i1, int i2) {
            return (c_x[i1] < c_x[i2]) * (bestAxis == 0) +
                   (c_y[i1] < c_y[i2]) * (bestAxis == 1) +
                   (c_z[i1] < c_z[i2]) * (bestAxis == 2);
        });

        // Create left and right node
        AzNode nl = {
            bestLab_min_x, bestLab_min_y, bestLab_min_z,
            bestLab_max_x, bestLab_max_y, bestLab_max_z,
            -1, -1, nd.ll, bestSplit, nd.depth + 1
        };

        AzNode nr = {
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