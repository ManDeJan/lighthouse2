#pragma once

#include "core_settings.h"

namespace lh2core {

class AABB {
public:
    int x, y, z, xd, yd, zd;
    AABB(int x, int y, int z, int xd, int yd, int zd) : x(x), y(y), z(z), xd(xd), yd(yd), zd(zd) {}
};

class Node {
    AABB bounds;
    int leftFirst;
    int count;

public:
    int getFirst() {
        return leftFirst;
    }
    bool isLeaf() {
        return count;
    }
    Node(int first, int count, AABB aabb) {}
    Node(int left, AABB aabb) {}
};

class BVH {
public:
    void ConstructBVH(Primitive *primitives);
    Node *root;

private:
    vector<Node> nodes;
    size_t nodeIndex;
};
} // namespace lh2core