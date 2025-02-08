#include <AsczBvh.cuh>

void HstNode::recalcMin(Flt3 v) {
    min.x = fminf(min.x, v.x);
    min.y = fminf(min.y, v.y);
    min.z = fminf(min.z, v.z);
}

void HstNode::recalcMax(Flt3 v) {
    max.x = fmaxf(max.x, v.x);
    max.y = fmaxf(max.y, v.y);
    max.z = fmaxf(max.z, v.z);
}

float HstNode::findCost() {
    return (max.x - min.x) * (max.y - min.y) * (max.z - min.z) * geoms.size();
}

float DevNode::hitDist(Flt3 rO, Flt3 rInvD) const {
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



_glb_ void calcAABB(
    Flt3 *ABmin, Flt3 *ABmax, Flt3 *gCent,
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
}

void AsczBvh::initAABB(AsczMesh &meshMgr) {
    h_ABmin.resize(meshMgr.gNum);
    h_ABmax.resize(meshMgr.gNum);
    h_gCent.resize(meshMgr.gNum);

    cudaMalloc(&d_ABmin, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_ABmax, meshMgr.gNum * sizeof(Flt3));
    cudaMalloc(&d_gCent, meshMgr.gNum * sizeof(Flt3));

    calcAABB<<<(meshMgr.gNum + 255) / 256, 256>>>(
        d_ABmin, d_ABmax, d_gCent,
        meshMgr.d_v, meshMgr.d_geom, meshMgr.gNum
    );
    cudaDeviceSynchronize();

    cudaMemcpy(
        h_ABmin.data(), d_ABmin, meshMgr.gNum * sizeof(Flt3),
        cudaMemcpyDeviceToHost
    );

    cudaMemcpy(
        h_ABmax.data(), d_ABmax, meshMgr.gNum * sizeof(Flt3),
        cudaMemcpyDeviceToHost
    );

    cudaMemcpy(
        h_gCent.data(), d_gCent, meshMgr.gNum * sizeof(Flt3),
        cudaMemcpyDeviceToHost
    );
}


void AsczBvh::toDevice() {
    nNum = h_dnodes.size();

    cudaMalloc(&d_nodes, h_dnodes.size() * sizeof(DevNode));
    cudaMemcpy(d_nodes, h_dnodes.data(), h_dnodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

    cudaMalloc(&d_gIdx, h_gIdx.size() * sizeof(int));
    cudaMemcpy(d_gIdx, h_gIdx.data(), h_gIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
}

void AsczBvh::buildBvh(HstNode *nodes, AsczMesh &meshMgr, int depth) {
    int nF = nodes->geoms.size();
    if (depth >= MAX_DEPTH || nF <= NODE_FACES) {
        nodes->leaf = true;
        return;
    }

    HstNode *left = new HstNode();
    HstNode *right = new HstNode();
    nodes->l = left;
    nodes->r = right;

    Flt3 AABBsize = nodes->max - nodes->min;

    float nodeCost = nodes->findCost();
    float curCost = nodeCost;

    #pragma omp parallel
    for (int x = 0; x < SPLIT_X; ++x) {
    for (int y = 0; y < SPLIT_Y; ++y) {
    for (int z = 0; z < SPLIT_Z; ++z) {
    for (int a = 0; a < 3; ++a) { // Axes
        HstNode l = HstNode();
        HstNode r = HstNode();

        Flt3 p = nodes->min + Flt3(
            AABBsize.x * (x + 1) / (SPLIT_X + 1),
            AABBsize.y * (y + 1) / (SPLIT_Y + 1),
            AABBsize.z * (z + 1) / (SPLIT_Z + 1)
        );

        #pragma omp parallel
        for (int i = 0; i < nF; ++i) {
            int idx = nodes->geoms[i];
            Flt3 center = h_gCent[idx];

            if (center[a] < p[a]) {
                l.geoms.push_back(idx);
                l.recalcMin(h_ABmin[idx]);
                l.recalcMax(h_ABmax[idx]);
            } else {
                r.geoms.push_back(idx);
                r.recalcMin(h_ABmin[idx]);
                r.recalcMax(h_ABmax[idx]);
            }
        }

        // Calculate cost
        int lF = l.geoms.size();
        int rF = r.geoms.size();

        float lCost = l.findCost();
        float rCost = r.findCost();
        float cost = lCost + rCost;

        if (cost < curCost) {
            curCost = cost;

            left->geoms = l.geoms;
            left->min = l.min;
            left->max = l.max;

            right->geoms = r.geoms;
            right->min = r.min;
            right->max = r.max;
        }
    }}}}

    // Unsuccesful split
    if (curCost >= nodeCost) {
        nodes->leaf = true;
        return;
    }

    int lF = left->geoms.size();
    int rF = right->geoms.size();

    buildBvh(left, meshMgr, depth + 1);
    buildBvh(right, meshMgr, depth + 1);
}

int AsczBvh::toShader(HstNode *node, std::vector<DevNode> &dnodes, std::vector<int> &fidx) {
    int idx = dnodes.size();
    dnodes.push_back(DevNode());

    dnodes[idx].min = node->min;
    dnodes[idx].max = node->max;

    if (node->leaf) {
        dnodes[idx].leaf = true;

        dnodes[idx].l = fidx.size();

        #pragma omp parallel for
        for (int i = 0; i < node->geoms.size(); ++i) {
            fidx.push_back(node->geoms[i]);
        }

        dnodes[idx].r = fidx.size();

        return idx;
    }

    dnodes[idx].l = toShader(node->l, dnodes, fidx);
    dnodes[idx].r = toShader(node->r, dnodes, fidx);

    return idx;
}

void AsczBvh::designBVH(AsczMesh &meshMgr) {
    const int &gNum = meshMgr.gNum;

    HstNode *root = new HstNode();
    root->geoms.resize(gNum);
    for (int i = 0; i < gNum; ++i) {
        root->geoms[i] = i;
        root->recalcMin(h_ABmin[i]);
        root->recalcMax(h_ABmax[i]);
    }

    buildBvh(root, meshMgr);
    toShader(root, h_dnodes, h_gIdx);
}