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



constexpr uint32_t expandBits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu; // Spread bits
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u; // Final interleave
    return v;
}

_glb_ void calcGeomSuperData(
    Flt3 *ABmin, Flt3 *ABmax, Flt3 *gCent, uint32_t *gMort, int *gIdx,
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

    // Calculate Morton code
    constexpr float grid = 1024.0f; // Increase for higher precision

    Flt3 scl = grid / (ABmax_ - ABmin_); 

    uint32_t x = static_cast<uint32_t>((gCent_.x - ABmin_.x) * scl.x);
    uint32_t y = static_cast<uint32_t>((gCent_.y - ABmin_.y) * scl.y);
    uint32_t z = static_cast<uint32_t>((gCent_.z - ABmin_.z) * scl.z);

    // Clamp
    x = x >= grid ? grid - 1 : x;
    y = y >= grid ? grid - 1 : y;
    z = z >= grid ? grid - 1 : z;

    gMort[idx] = (expandBits(x) << 2) | (expandBits(y) << 1) | expandBits(z);

    ABmin[idx] = ABmin_;
    ABmax[idx] = ABmax_;
    gCent[idx] = gCent_;

    gIdx[idx] = idx;
}

void AsczBvh::initAABB(AsczMesh &meshMgr) {
    h_ABmin.resize(meshMgr.gNum);
    h_ABmax.resize(meshMgr.gNum);
    h_gCent.resize(meshMgr.gNum);
    h_gMort.resize(meshMgr.gNum);
    h_gIdx.resize(meshMgr.gNum);

    cudaMalloc(&d_ABmin, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_ABmax, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_gCent, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_gMort, meshMgr.gNum * sizeof(uint32_t));
    cudaMalloc(&d_gIdx, meshMgr.gNum * sizeof(int));

    calcGeomSuperData<<<(meshMgr.gNum + 255) / 256, 256>>>(
        d_ABmin, d_ABmax, d_gCent, d_gMort, d_gIdx,
        meshMgr.d_v, meshMgr.d_geom, meshMgr.gNum
    );
    cudaDeviceSynchronize();

    cudaMemcpy(h_ABmin.data(), d_ABmin, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_ABmax.data(), d_ABmax, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_gCent.data(), d_gCent, meshMgr.gNum * sizeof(Flt3), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_gMort.data(), d_gMort, meshMgr.gNum * sizeof(uint32_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_gIdx.data(), d_gIdx, meshMgr.gNum * sizeof(int), cudaMemcpyDeviceToHost);

    std::sort(h_gIdx.begin(), h_gIdx.end(), [&](int a, int b) {
        return h_gMort[a] < h_gMort[b];
    });
}


void AsczBvh::toDevice() {
    nNum = h_nodes.size();

    cudaMalloc(&d_nodes, h_nodes.size() * sizeof(DevNode));
    cudaMemcpy(d_nodes, h_nodes.data(), h_nodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

    cudaMalloc(&d_gIdx, h_gIdx.size() * sizeof(int));
    cudaMemcpy(d_gIdx, h_gIdx.data(), h_gIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
}


int AsczBvh::buildBvh(DevNode &node, int depth) {
    h_nodes.push_back(node);

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
        std::sort(h_gIdx.begin() + node.ll, h_gIdx.begin() + node.lr, [&](int i1, int i2) {
            return h_gCent[i1][a] < h_gCent[i2][a];
        });

        for (int b = 0; b < BIN_COUNT; ++b) {
            DevNode l, r;

            float splitPoint = node.min[a] + AABBsize[a] * (b + 1) / BIN_COUNT;

            int splitIdx = node.ll;

            for (int g = node.ll; g < node.lr; ++g) {
                if (h_gCent[h_gIdx[g]][a] < splitPoint) {
                    l.recalcMin(h_ABmin[h_gIdx[g]]);
                    l.recalcMax(h_ABmax[h_gIdx[g]]);
                    splitIdx++;
                }
                else {
                    r.recalcMin(h_ABmin[h_gIdx[g]]);
                    r.recalcMax(h_ABmax[h_gIdx[g]]);
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

    std::sort(h_gIdx.begin() + node.ll, h_gIdx.begin() + node.lr, [&](int i1, int i2) {
        return h_gCent[i1][bestAxis] < h_gCent[i2][bestAxis];
    });

    DevNode l = { bestLMin, bestLMax, -1, -1, node.ll, bestSplit };
    DevNode r = { bestRMin, bestRMax, -1, -1, bestSplit, node.lr };

    int curIdx = h_nodes.size() - 1;

    h_nodes[curIdx].cl = h_nodes.size();
    idx += buildBvh(l, depth + 1);

    h_nodes[curIdx].cr = h_nodes.size();
    idx += buildBvh(r, depth + 1);

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

    buildBvh(root, 0);

    // Debug
    for (int i = 0; i < h_nodes.size(); ++i) {
        printf("Node %d: %d %d %d %d\n", i, h_nodes[i].cl, h_nodes[i].cr, h_nodes[i].ll, h_nodes[i].lr);
    }
}