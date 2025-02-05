#include <BvhManager.cuh>

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
    return (max.x - min.x) * (max.y - min.y) * (max.z - min.z) * faces.size();
}

float DevNode::hitDist(Flt3 rO, Flt3 rInvD) const {
    // If origin is inside the AABB
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


void BvhManager::toDevice() {
    nNum = h_dnodes.size();

    cudaMalloc(&d_nodes, h_dnodes.size() * sizeof(DevNode));
    cudaMemcpy(d_nodes, h_dnodes.data(), h_dnodes.size() * sizeof(DevNode), cudaMemcpyHostToDevice);

    cudaMalloc(&d_fidx, h_fidx.size() * sizeof(int));
    cudaMemcpy(d_fidx, h_fidx.data(), h_fidx.size() * sizeof(int), cudaMemcpyHostToDevice);
}

void BvhManager::buildBvh(HstNode *nodes, MeshManager &meshMgr, int depth) {
    const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
    const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max
    const Vecs3f &fABcen = meshMgr.h_fABcen; // Face's AABB center

    int nF = nodes->faces.size();
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
            int idx = nodes->faces[i];
            Flt3 center = fABcen[idx];

            if (center[a] < p[a]) {
                l.faces.push_back(idx);
                l.recalcMin(fABmin[idx]);
                l.recalcMax(fABmax[idx]);
            } else {
                r.faces.push_back(idx);
                r.recalcMin(fABmin[idx]);
                r.recalcMax(fABmax[idx]);
            }
        }

        // Calculate cost
        int lF = l.faces.size();
        int rF = r.faces.size();

        float lCost = l.findCost();
        float rCost = r.findCost();
        float cost = lCost + rCost;

        if (cost < curCost) {
            curCost = cost;

            left->faces = l.faces;
            left->min = l.min;
            left->max = l.max;

            right->faces = r.faces;
            right->min = r.min;
            right->max = r.max;
        }
    }}}}

    // Unsuccesful split
    if (curCost >= nodeCost) {
        nodes->leaf = true;
        return;
    }

    int lF = left->faces.size();
    int rF = right->faces.size();

    buildBvh(left, meshMgr, depth + 1);
    buildBvh(right, meshMgr, depth + 1);
}

int BvhManager::toShader(HstNode *node, std::vector<DevNode> &dnodes, std::vector<int> &fidx) {
    int idx = dnodes.size();
    dnodes.push_back(DevNode());

    dnodes[idx].min = node->min;
    dnodes[idx].max = node->max;

    if (node->leaf) {
        dnodes[idx].leaf = true;

        dnodes[idx].l = fidx.size();

        #pragma omp parallel for
        for (int i = 0; i < node->faces.size(); ++i) {
            fidx.push_back(node->faces[i]);
        }

        dnodes[idx].r = fidx.size();

        return idx;
    }

    dnodes[idx].l = toShader(node->l, dnodes, fidx);
    dnodes[idx].r = toShader(node->r, dnodes, fidx);

    return idx;
}

void BvhManager::designBVH(MeshManager &meshMgr) {
    const Vecs3i &fv = meshMgr.h_fv; // Faces
    const Vecs3f &fABmin = meshMgr.h_fABmin; // Face's AABB min
    const Vecs3f &fABmax = meshMgr.h_fABmax; // Face's AABB max

    HstNode *root = new HstNode();
    root->faces.resize(fv.size());
    for (int i = 0; i < fv.size(); ++i) {
        root->faces[i] = i;
        root->recalcMin(fABmin[i]);
        root->recalcMax(fABmax[i]);
    }

    buildBvh(root, meshMgr);
    toShader(root, h_dnodes, h_fidx);
}