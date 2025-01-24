#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <cassert>

#include <cmath>

struct float3 {
    float x, y, z;
};

struct Triangle {
    int id = -1;
    float3 v0, v1, v2;

    Triangle(float3 v0, float3 v1, float3 v2) : v0(v0), v1(v1), v2(v2) {}
    Triangle(int id, float3 v0, float3 v1, float3 v2) : id(id), v0(v0), v1(v1), v2(v2) {}
};

struct Node {
    float3 ABmin, ABmax;
    Node* left;
    Node* right;

    bool leaf = false;
    std::vector<Triangle> triangles;
};

void createBVH(Node *node, std::vector<Triangle> tris, float3 ABmin, float3 ABmax) {
    // For the time being just split the bvh in half
    // based on the longest axis

    // Find the longest axis
    float3 diff = {ABmax.x - ABmin.x, ABmax.y - ABmin.y, ABmax.z - ABmin.z};
    float maxDiff = std::max({diff.x, diff.y, diff.z});

    // Split the axis in half
    float3 mid = {ABmin.x + diff.x / 2, ABmin.y + diff.y / 2, ABmin.z + diff.z / 2};

    // Create the left and right nodes
    Node *left = new Node();
    Node *right = new Node();
}


int main() {
    // Create some triangles with varying coord
    std::vector<Triangle> triangles = {
        Triangle(0, {0, 0, 2.5}, {10.6, 0, 0}, {0, 12.1, 0}),
        Triangle(1, {0, 1.5, 0}, {0, 0, 0}, {0, 0, 1.5}),
        Triangle(2, {0, 0, 0}, {1.5, 0, 0}, {0, 1.5, 0}),
        Triangle(3, {0, 0, 0}, {0, 1.5, 0}, {1.5, 0, 0}),
        Triangle(4, {0, 0, 0}, {0, 0, 1.5}, {1.5, 0, 0}),
        Triangle(5, {12.1, 0, 0}, {0, 0, 2.5}, {10.6, 0, 0}),
    };

    Node *root = new Node();
}