#include <AsczBvh.cuh>

#include <algorithm>

float DevNode::hitDist(const Flt3 &rO, const Flt3 &rInvD) const {
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

void DevNode::recalcMin(const Flt3 &v) {
    min.x = fminf(min.x, v.x);
    min.y = fminf(min.y, v.y);
    min.z = fminf(min.z, v.z);
}

void DevNode::recalcMax(const Flt3 &v) {
    max.x = fmaxf(max.x, v.x);
    max.y = fmaxf(max.y, v.y);
    max.z = fmaxf(max.z, v.z);
}



_glb_ void calcGeomSuperData(
    Flt3 *ABmin, Flt3 *ABmax, Flt3 *gCent, int *gIdx,
    Flt3 *mv, AzGeom *geom, int gNum
) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= gNum) return;

    Flt3 ABmin_ = Flt3(INFINITY);
    Flt3 ABmax_ = Flt3(-INFINITY);
    Flt3 gCent_ = Flt3();

    AzGeom &g = geom[idx];

    switch (g.type)
    {
    case AzGeom::TRIANGLE:
        for (int j = 0; j < 3; ++j) {
            Flt3 v = mv[g.tri.v[j]];

            ABmin_.x = fminf(ABmin_.x, v.x);
            ABmin_.y = fminf(ABmin_.y, v.y);
            ABmin_.z = fminf(ABmin_.z, v.z);

            ABmax_.x = fmaxf(ABmax_.x, v.x);
            ABmax_.y = fmaxf(ABmax_.y, v.y);
            ABmax_.z = fmaxf(ABmax_.z, v.z);

            gCent_ += v;
        }

        gCent_ /= 3;
        break;

    case AzGeom::SPHERE:
        Flt3 c = mv[g.sph.c];
        ABmin_ = c - g.sph.r;
        ABmax_ = c + g.sph.r;
        gCent_ = c;
        break;
    }

    ABmin[idx] = ABmin_;
    ABmax[idx] = ABmax_;
    gCent[idx] = gCent_;
    gIdx[idx] = idx;
}

void AsczBvh::initAABB(AsczMesh &meshMgr) {
    h_ABmin.resize(meshMgr.gNum);
    h_ABmax.resize(meshMgr.gNum);
    h_gCent.resize(meshMgr.gNum);
    h_gIdx.resize(meshMgr.gNum);

    cudaMalloc(&d_ABmin, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_ABmax, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_gCent, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_gIdx, meshMgr.gNum * sizeof(int));

    calcGeomSuperData<<<(meshMgr.gNum + 255) / 256, 256>>>(
        d_ABmin, d_ABmax, d_gCent, d_gIdx,
        meshMgr.d_v, meshMgr.d_geom, meshMgr.gNum
    );
    cudaDeviceSynchronize();

    cudaMemcpy(h_ABmin.data(), d_ABmin, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_ABmax.data(), d_ABmax, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_gCent.data(), d_gCent, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_gIdx.data(), d_gIdx, meshMgr.gNum * sizeof(int), cudaMemcpyDeviceToHost);
}


void AsczBvh::toDevice() {
    nNum = h_nodes.size();

    cudaMalloc(&d_nodes, h_nodes.size() * sizeof(DevNode));
    cudaMemcpy(d_nodes, h_nodes.data(), h_nodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

    cudaMalloc(&d_gIdx, h_gIdx.size() * sizeof(int));
    cudaMemcpy(d_gIdx, h_gIdx.data(), h_gIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
}


int AsczBvh::buildBvh(
    VecNode &allNode, VecI &allGIdx, DevNode &node, int depth,
    const Vec3f &ABmin, const Vec3f &ABmax, const Vec3f &gCent,
    const int MAX_DEPTH, const int NODE_FACES, const int BIN_COUNT
) {
    allNode.push_back(node);

    int idx = 0;

    int nG = node.lr - node.ll;
    if (nG <= NODE_FACES || depth >= MAX_DEPTH) {
        node.cl = -1;
        node.cr = -1;
        return 1;
    }

    Flt3 AABBsize = node.max - node.min;
    float curCost = DevNode::findCost(node.min, node.max, nG);

    int bestAxis = -1;
    int bestSplit = -1;
    Flt3 bestLMin, bestLMax;
    Flt3 bestRMin, bestRMax;
    float bestCost = curCost;

    for (int a = 0; a < 3; ++a) {
        std::sort(allGIdx.begin() + node.ll, allGIdx.begin() + node.lr, [&](int i1, int i2) {
            return gCent[i1][a] < gCent[i2][a];
        });

        for (int b = 0; b < BIN_COUNT; ++b) {
            DevNode l, r;

            float splitPoint = node.min[a] + AABBsize[a] * (b + 1) / BIN_COUNT;

            int splitIdx = node.ll;

            #pragma omp parallel
            for (int g = node.ll; g < node.lr; ++g) {
                int i = allGIdx[g];

                float cent = gCent[i][a];

                if (cent < splitPoint) {
                    l.recalcMin(ABmin[i]);
                    l.recalcMax(ABmax[i]);
                    splitIdx++;
                }
                else {
                    r.recalcMin(ABmin[i]);
                    r.recalcMax(ABmax[i]);
                }
            }

            float lCost = DevNode::findCost(l.min, l.max, splitIdx - node.ll);
            float rCost = DevNode::findCost(r.min, r.max, node.lr - splitIdx);
            float cost = lCost + rCost;

            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = a;
                bestSplit = splitIdx;

                bestLMin = l.min; bestLMax = l.max;
                bestRMin = r.min; bestRMax = r.max;
            }
        }
    }

    if (bestSplit == -1 || bestAxis == -1) {
        node.cl = -1;
        node.cr = -1;
        return 1;
    }

    std::sort(allGIdx.begin() + node.ll, allGIdx.begin() + node.lr, [&](int i1, int i2) {
        return gCent[i1][bestAxis] < gCent[i2][bestAxis];
    });

    DevNode l = { bestLMin, bestLMax, -1, -1, node.ll, bestSplit };
    DevNode r = { bestRMin, bestRMax, -1, -1, bestSplit, node.lr };

    int curIdx = allNode.size() - 1;

    allNode[curIdx].cl = allNode.size();
    idx += buildBvh(
        allNode, allGIdx, l, depth + 1,
        ABmin, ABmax, gCent,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    allNode[curIdx].cr = allNode.size();
    idx += buildBvh(
        allNode, allGIdx, r, depth + 1,
        ABmin, ABmax, gCent,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );

    return idx + 1;
}

void AsczBvh::designBVH(AsczMesh &meshMgr) {
    const int &gNum = meshMgr.gNum;

    DevNode root = { Flt3(INFINITY), Flt3(-INFINITY), -1, -1, 0, gNum };
    // Calculate the root's AABB
    for (int i = 0; i < gNum; ++i) {
        root.recalcMin(h_ABmin[i]);
        root.recalcMax(h_ABmax[i]);
    }

    buildBvh(
        h_nodes, h_gIdx, root, 0,
        h_ABmin, h_ABmax, h_gCent,
        MAX_DEPTH, NODE_FACES, BIN_COUNT
    );
}