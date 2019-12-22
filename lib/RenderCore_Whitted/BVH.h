#pragma once

#include "core_settings.h"

namespace lh2core {

class AABB {
public:
    float3 minBounds =
        make_float3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    float3 maxBounds =
        make_float3(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());

    AABB(float3 minBounds, float3 maxBounds) : minBounds(minBounds), maxBounds(maxBounds) {}
    AABB() = default;
};


class Bin{
public:
    AABB bounds;
    int count = 0;
    vector<uint> primIndices;
    float cost;
    
	Bin() = default;

	void addPrim(uint primIndex) {
        count++;
        primIndices.push_back(primIndex);
	}
    void evaluateBounds();
    AABB evaluateGetBounds();
};

class Node {
    int leftFirst;

    void partition();
    void binnedPartition();
    void convertNode(vector<uint> left, AABB leftAABB, vector<uint> right, AABB rightAABB);

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
