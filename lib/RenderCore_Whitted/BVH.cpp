#include "BVH.h"

namespace lh2core {

void BVH::constructBVH(vector<CoreTri> &primitives, size_t N) {

    // create index array
   // indices = new uint[N];
   // for (int i = 0; i < N; i++) indices[i] = i;
    // allocate BVH root node
    nodes.clear();
    nodes.resize(N * 2 – 1);
    nodeIndex = 0;
	root = &nodes[0];
    // subdivide root node
    root->setFirst(0);
    root->setCount(N);
    root->setBounds(calculateBounds(root->first, root->count));
    root->subdivide();
}

void Node::subdivide() {	
    if (count < 3) return;

    partition();

    nodeIndex += 2;
    nodes[left()].subdivide();
    nodes[right()].subdivide();
}

void Node::partition() 
{
    int split = count / 2;
    nodes[nodeIndex+1] = new Node(first(), split);//left node
    nodes[nodeIndex+2] = new Node(split, count);//right node

	setLeft(nodeIndex+1);
	count = 0;
}
AABB calculateBounds(int first, int count) {
	
	int x, y, z, xd, xyd, zd;
	//somehow calculate bounds based on the primitves
    for (int i = first; i <= count; i++) { 
		primitives[i];
	}
    return new AABB;
}
} // namespace lh2core