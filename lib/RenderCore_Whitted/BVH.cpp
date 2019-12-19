#include "BVH.h"

namespace lh2core {

void BVH::constructBVH(vector<CoreTri> &primitives, size_t N) {
    // create index array
	
    indices.resize(N);
    for (int i = 0; i < N; i++) indices[i] = i;
    // allocate BVH root node
    nodes.clear();
    nodes.resize(N * 2 - 1);
    nodeIndex = 0;
    root = Node();
    // subdivide root node
    root->setFirst(0);
    root->setCount(N);
    root->setBounds(calculateBounds(root->first(), root->count));
    root->subdivide();
}

void Node::subdivide() {
    if (count < 3) return;

    partition();

    nodeIndex += 2;
    nodes[left()].subdivide();
    nodes[right()].subdivide();
}

void Node::partition() {

	float bestSplitCost;
	vector<uint> bestSplitLeft;
    vector<uint> bestSplitRight;

	for (int i = 0; i < count; i++) { 
		CoreTri primSplit = primitives[indices[first() + i]];
        float3 primSplitCenter = triangleCenter(primSplit);

        vector<uint> splitLeft;
        vector<uint> splitRight;
		
		// X
		//split left and right on X
		for (int j = 0; j < count; j++) { 
			uint indexj = indices[first() + j];
			CoreTri prim = primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.x < primSplitCenter.x) 
				splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
		}
		
		//calculate cost on X
        float cost = calculateSAH(splitLeft) + calculateSAH(splitRight);
		if (cost < bestSplitCost) { bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
		}

		// Y
        splitLeft.clear();
        splitRight.clear();
		//split left and right on Y
        for (int j = 0; j < count; j++) {
            uint indexj = indices[first() + j];
            CoreTri prim = primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.y < primSplitCenter.y) splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
        }

        //calculate cost on Y
        float cost = calculateSAH(splitLeft) + calculateSAH(splitRight);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
        }

        // Z
        splitLeft.clear();
        splitRight.clear();
        //split left and right on Z
        for (int j = 0; j < count; j++) {
            uint indexj = indices[first() + j];
            CoreTri prim = primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.z < primSplitCenter.z) splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
        }

        //calculate cost on Z
        float cost = calculateSAH(splitLeft) + calculateSAH(splitRight);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
        }
	}


	//TODO:
	//write bestleftsplit to left side of indices and bestrightsplit to right side of indices
	//initialize left and right node
	//rewrite current node to non-leaf node
}

float3 triangleCenter(CoreTri &tri) {
    return (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;
}

float calculateSAH(vector<uint> indices) {
    float3 minBounds = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
    float3 maxBounds = make_float3(FLT_MIN, FLT_MIN, FLT_MIN);

	//calculate bounds
	for (uint index : indices) {
        CoreTri &primitive = primitives[index];
        float3 primMin = fminf(fminf(primitive.vertex0, primitive.vertex1), primitive.vertex2);
        float3 primMax = fmaxf(fmaxf(primitive.vertex0, primitive.vertex1), primitive.vertex2);

        minBounds = fminf(minBounds, primMin);
        maxBounds = fmaxf(maxBounds, primMax);
	}

    float3 box = maxBounds - minBounds;
	return indices.size * (2 * box.x * box.y + 2 * box.y * box.z + 2 * box.z * box.x);
}

AABB BVH::calculateBounds(int first, int count) {
    float3 minBounds = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
    float3 maxBounds = make_float3(FLT_MIN, FLT_MIN, FLT_MIN);
    //somehow calculate bounds based on the primitves
    for (int i = first; i <= count; i++) { 
		CoreTri &primitive = primitives[indices[i]]; 
		float3 primMin = fminf(fminf(primitive.vertex0, primitive.vertex1), primitive.vertex2);
        float3 primMax = fmaxf(fmaxf(primitive.vertex0, primitive.vertex1), primitive.vertex2);

		minBounds = fminf(minBounds, primMin);
        maxBounds = fmaxf(maxBounds, primMax);
	}
    return AABB(minBounds, maxBounds);
}
} // namespace lh2core
