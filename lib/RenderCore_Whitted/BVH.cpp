

#include "BVH.h"


namespace lh2core {

void BVH::ConstructBVH(Primitive *primitives) {
    // create index array
    indices = new uint[N];
    for (int i = 0; i < N; i++) indices[i] = i;
    // allocate BVH root node
    nodes.clear();
    nodes.resize(N * 2 – 1);
    root = &nodes[0];
    nodeIndex = 2;
    // subdivide root node
    root->first = 0;
    root->count = N;
    root->bounds = CalculateBounds(primitives, root->first, root->count);
    root->Subdivide();
}

void BVHNode::Subdivide() {
    if (count < 3) return;
    this.left = pool[poolPtr++];
    this.right = pool[poolPtr++];
    Partition();
    this.left->Subdivide();
    this.right->Subdivide();
    this.isLeaf = false;
}

} // namespace lh2core