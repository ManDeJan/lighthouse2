#pragma once

#include "core_settings.h"

namespace lh2core {

class AABB {
public:
    float3 minBounds, maxBounds;
    AABB(float3 minBounds, float3 maxBounds) : minBounds(minBounds), maxBounds(maxBounds) {}
    AABB() = default;
};

class Node {
    int leftFirst;

public:
    int count;
    AABB bounds;

    Node() = default;
    //leaf node
    Node(int first, int count, AABB bounds) : leftFirst(first), count(count), bounds(bounds) {}
    Node(int left, AABB bounds) : leftFirst(left), bounds(bounds) {} //node

    void setFirst(int first) {
        leftFirst = first;
    }
    void setLeft(int left) {
        leftFirst = left;
    }
    int first() {
        return leftFirst;
    }
    int left() {
        return leftFirst;
    }
    int right() {
        return leftFirst + 1;
    }

    void setCount(int count) {
        this->count = count;
    }

    void setBounds(AABB bounds) {
        this->bounds = bounds;
    }

    bool isLeaf() {
        return count;
    }

    void subdivide();

    void partition();
};

class Mesh;

class BVH {
public:
    static Node *root;
    static vector<Node> nodes;
    static size_t nodeIndex;
    static vector<uint> indices;
    static vector<CoreTri> primitives;

    void setMesh(Mesh &mesh);
    void constructBVH();
};

// uint indices[1];
} // namespace lh2core
