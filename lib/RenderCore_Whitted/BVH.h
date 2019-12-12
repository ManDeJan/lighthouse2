#pragma once

#include "core_settings.h"

namespace lh2core {

class AABB {
public:
    float x, y, z, xd, yd, zd;
    AABB(float x, float y, float z, float xd, float yd, float zd) : x(x), y(y), z(z), xd(xd), yd(yd), zd(zd) {}
};

AABB calculateBounds(int first, int count);

class Node {
    AABB bounds;
    int leftFirst;

public:
    int count;

    //leaf node
    Node(int first, int count) : leftFirst(first), count(count) {
        bounds = calculateBounds(first, count);
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
    const Mesh &primitives;
    
public:
    BVH(Mesh &primitives) : primitives(primitives) {}
    Node *root;
};

vector<Node> nodes;
size_t nodeIndex;
} // namespace lh2core
