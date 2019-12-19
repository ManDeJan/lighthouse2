#pragma once

#include "core_settings.h"

namespace lh2core {

class AABB {
public:
    float3 minBounds, maxBounds;
    AABB(float3 minBounds, float3 maxBounds) : minBounds(minBounds), maxBounds(maxBounds) {}
    AABB() = default;
};

AABB calculateBounds(int first, int count);

class Node {
    AABB bounds;
    int leftFirst;

public:
    int count;

    Node() = default;
    //leaf node
    Node(int first, int count) : leftFirst(first), count(count), bounds(calculateBounds(first, count)) {
        // bounds = calculateBounds(first, count);
    }
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

class BVH {
public:
    static const Mesh &primitives;
    static Node *root;
    static vector<Node> nodes;
    static vector<uint> indices;
    static vector<CoreTri> primitives;

    BVH(Mesh &primitives) : primitives(primitives) {}
    void constructBVH(vector<CoreTri> &primitives, size_t N);

    Node *root;
};

// size_t nodeIndex;
// uint indices[1];
} // namespace lh2core
