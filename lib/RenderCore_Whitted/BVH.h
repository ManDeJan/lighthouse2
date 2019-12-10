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


public:
    int count;

	//leaf node
    Node(int first, int count) : leftFirst(first), count(count) {
        calculateBounds(first, count);
    
	}                                                           
    Node(int left, AABB bounds) : leftFirst(left), bounds(bounds) {}                            //node
    
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


	void subdivide() {

	}

	void partition();
};

class BVH {
public:
    void constructBVH(vector<CoreTri> *primitives, size_t N);
    Node *root;

private:

};

vector<Node> nodes;
size_t nodeIndex;
} // namespace lh2core