#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <cassert>

#include <cmath>

struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vec3 operator-(const Vec3 &other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    float dot(const Vec3 &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    float length() const {
        return sqrt(x * x + y * y + z * z);
    }

    float max_component() const {
        return std::max(x, std::max(y, z));
    }
};

struct AABB {
    Vec3 min, max;

    AABB() : min(Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max())),
             max(Vec3(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest())) {}

    void expand(const Vec3 &point) {
        min = Vec3(std::min(min.x, point.x), std::min(min.y, point.y), std::min(min.z, point.z));
        max = Vec3(std::max(max.x, point.x), std::max(max.y, point.y), std::max(max.z, point.z));
    }

    void expand(const AABB &other) {
        expand(other.min);
        expand(other.max);
    }

    float surfaceArea() const {
        Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vec3 centroid() const {
        return Vec3((min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2);
    }

    int max_component() const {
        Vec3 extents = max - min;
        if (extents.x >= extents.y && extents.x >= extents.z) return 0;
        if (extents.y >= extents.x && extents.y >= extents.z) return 1;
        return 2;
    }
};

struct Triangle {
    Vec3 v0, v1, v2;

    Triangle(const Vec3 &_v0, const Vec3 &_v1, const Vec3 &_v2) : v0(_v0), v1(_v1), v2(_v2) {}

    AABB getAABB() const {
        AABB bbox;
        bbox.expand(v0);
        bbox.expand(v1);
        bbox.expand(v2);
        return bbox;
    }
};

struct BVHNode {
    AABB bbox;
    int left, right;
    bool isLeaf;
    int start, count;

    BVHNode() : left(-1), right(-1), isLeaf(false), start(-1), count(0) {}
};

// Partitioning the triangles based on an axis (X, Y, Z)
int partition(std::vector<Triangle> &triangles, std::vector<int> &triangleIndices, int start, int end, int axis) {
    auto compare = [&](int i, int j) {
        Vec3 centroid_i = triangles[i].getAABB().centroid();
        Vec3 centroid_j = triangles[j].getAABB().centroid();
        if (axis == 0) return centroid_i.x < centroid_j.x;
        if (axis == 1) return centroid_i.y < centroid_j.y;
        return centroid_i.z < centroid_j.z;
    };
    std::sort(triangleIndices.begin() + start, triangleIndices.begin() + end, compare);

    AABB leftBBox, rightBBox;
    for (int i = start; i < end; ++i) {
        leftBBox.expand(triangles[triangleIndices[i]].getAABB());
    }

    float bestCost = std::numeric_limits<float>::infinity();
    int bestPartition = -1;

    for (int i = start + 1; i < end; ++i) {
        rightBBox.expand(triangles[triangleIndices[i - 1]].getAABB());
        float cost = leftBBox.surfaceArea() * (i - start) + rightBBox.surfaceArea() * (end - i);
        if (cost < bestCost) {
            bestCost = cost;
            bestPartition = i;
        }
    }

    return bestPartition;
}

// Building the BVH recursively
void buildBVH(std::vector<Triangle> &triangles, std::vector<int> &triangleIndices, int start, int end, std::vector<BVHNode> &bvh) {
    if (end - start <= 0) return;

    BVHNode node;
    AABB nodeBBox;
    for (int i = start; i < end; ++i) {
        nodeBBox.expand(triangles[triangleIndices[i]].getAABB());
    }

    if (end - start == 1) {
        node.isLeaf = true;
        node.start = start;
        node.count = end - start;
        node.bbox = nodeBBox;
    } else {
        int axis = nodeBBox.max_component();
        int parti = partition(triangles, triangleIndices, start, end, axis);

        node.isLeaf = false;
        node.bbox = nodeBBox;

        int leftChild = bvh.size();
        bvh.push_back(BVHNode());
        buildBVH(triangles, triangleIndices, start, parti, bvh);
        node.left = leftChild;

        int rightChild = bvh.size();
        bvh.push_back(BVHNode());
        buildBVH(triangles, triangleIndices, parti, end, bvh);
        node.right = rightChild;
    }

    bvh.push_back(node);
}

int main() {
    // Example triangles
    std::vector<Triangle> triangles = {
        Triangle(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0)),
        Triangle(Vec3(1, 1, 1), Vec3(2, 1, 1), Vec3(1, 2, 1)),
        Triangle(Vec3(0, 2, 2), Vec3(1, 2, 2), Vec3(0, 3, 2)),
        Triangle(Vec3(2, 2, 2), Vec3(3, 2, 2), Vec3(2, 3, 2))
    };

    // Initialize triangle indices
    std::vector<int> triangleIndices(triangles.size());
    for (int i = 0; i < triangles.size(); ++i) {
        triangleIndices[i] = i;
    }

    // BVH construction
    std::vector<BVHNode> bvh;
    buildBVH(triangles, triangleIndices, 0, triangles.size(), bvh);

    // Output the BVH structure
    std::cout << "BVH constructed with " << bvh.size() << " nodes." << std::endl;

    // Print some details of the BVH nodes for verification
    for (size_t i = 0; i < bvh.size(); ++i) {
        const BVHNode &node = bvh[i];
        if (node.isLeaf) {
            std::cout << "Leaf Node " << i << ": Start = " << node.start << ", Count = " << node.count << std::endl;
        } else {
            std::cout << "Internal Node " << i << ": Left = " << node.left << ", Right = " << node.right << std::endl;
        }
    }

    return 0;
}
