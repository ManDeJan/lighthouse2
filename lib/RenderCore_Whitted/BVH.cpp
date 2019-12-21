#include "BVH.h"

namespace lh2core {

Node *BVH::root;
vector<Node> BVH::nodes;
size_t BVH::nodeIndex;
vector<uint> BVH::indices;
vector<CoreTri> BVH::primitives;

float3 triangleCenter(CoreTri &tri) {
    return (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;
}

float calculateSAH(AABB bounds) {
    float3 box = bounds.maxBounds - bounds.minBounds;
    return BVH::indices.size() * (2 * box.x * box.y + 2 * box.y * box.z + 2 * box.z * box.x);
}

AABB calculateBounds(const vector<uint> &indices) {
    float3 minBounds =
        make_float3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    float3 maxBounds =
        make_float3(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());

    //calculate bounds
    for (uint index : indices) {
        CoreTri &primitive = BVH::primitives[index];
        float3 primMin = fminf(fminf(primitive.vertex0, primitive.vertex1), primitive.vertex2);
        float3 primMax = fmaxf(fmaxf(primitive.vertex0, primitive.vertex1), primitive.vertex2);

        minBounds = fminf(minBounds, primMin);
        maxBounds = fmaxf(maxBounds, primMax);
    }
    return AABB(minBounds, maxBounds);
}

// AABB BVH::calculateBounds(int first, int count) {
//     float3 minBounds = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
//     float3 maxBounds = make_float3(FLT_MIN, FLT_MIN, FLT_MIN);
//     //somehow calculate bounds based on the primitves
//     for (int i = first; i <= count; i++) {
//         CoreTri &primitive = BVH::primitives[indices[i]];
//         float3 primMin = fminf(fminf(primitive.vertex0, primitive.vertex1), primitive.vertex2);
//         float3 primMax = fmaxf(fmaxf(primitive.vertex0, primitive.vertex1), primitive.vertex2);

//         minBounds = fminf(minBounds, primMin);
//         maxBounds = fmaxf(maxBounds, primMax);
//     }
//     return AABB(minBounds, maxBounds);
// }

void BVH::constructBVH() {
    // create index array

    size_t N = primitives.size();
    indices.resize(N);
    for (int i = 0; i < N; i++) indices[i] = i;
    // allocate BVH root node
    nodes.clear();
    nodes.resize(N * 2 - 1);
    nodeIndex = 0;
    root = &nodes[0];
    // subdivide root node
    root->setFirst(0);
    root->setCount(N);
    root->setBounds(calculateBounds(indices));
    root->subdivide();
}

void Node::subdivide() {
    if (count < 3) return;

    partition();

    BVH::nodeIndex += 2;
    BVH::nodes[left()].subdivide();
    BVH::nodes[right()].subdivide();
}

void Node::partition() {
    float bestSplitCost;
    vector<uint> bestSplitLeft;
    vector<uint> bestSplitRight;
    AABB bestBSL, bestBSR;

    for (int i = 0; i < count; i++) {
        CoreTri primSplit = BVH::primitives[BVH::indices[first() + i]];
        float3 primSplitCenter = triangleCenter(primSplit);

        vector<uint> splitLeft;
        vector<uint> splitRight;
        AABB boundsSL;
        AABB boundsSR;

        // X
        //split left and right on X
        for (int j = 0; j < count; j++) {
            uint indexj = BVH::indices[first() + j];
            CoreTri prim = BVH::primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.x < primSplitCenter.x) splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
        }

        //calculate cost on X
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        float cost = calculateSAH(boundsSL) + calculateSAH(boundsSR);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
            bestBSL = boundsSL;
            bestBSR = boundsSR;
        }

        // Y
        splitLeft.clear();
        splitRight.clear();
        //split left and right on Y
        for (int j = 0; j < count; j++) {
            uint indexj = BVH::indices[first() + j];
            CoreTri prim = BVH::primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.y < primSplitCenter.y) splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
        }

        //calculate cost on Y
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        cost = calculateSAH(boundsSL) + calculateSAH(boundsSR);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
            bestBSL = boundsSL;
            bestBSR = boundsSR;
        }

        // Z
        splitLeft.clear();
        splitRight.clear();
        //split left and right on Z
        for (int j = 0; j < count; j++) {
            uint indexj = BVH::indices[first() + j];
            CoreTri prim = BVH::primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.z < primSplitCenter.z) splitLeft.push_back(indexj);
            else
                bestSplitRight.push_back(indexj);
        }

        //calculate cost on Z
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        cost = calculateSAH(boundsSL) + calculateSAH(boundsSR);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
            bestBSL = boundsSL;
            bestBSR = boundsSR;
        }
    }

    //Create left node
    for (int i = 0; i < bestSplitLeft.size(); i++) BVH::indices[first() + i] = bestSplitLeft[i];
    BVH::nodes[BVH::nodeIndex + 1] = Node(first(), bestSplitLeft.size(), bestBSL);

    //create right node
    for (int i = 0; i < bestSplitRight.size(); i++)
        BVH::indices[first() + bestSplitLeft.size() + i] = bestSplitRight[i];
    BVH::nodes[BVH::nodeIndex + 2] = Node(first() + bestSplitLeft.size(), bestSplitRight.size(), bestBSR);

    //setLeftNode and set to parent node
    setLeft(BVH::nodeIndex + 1);
    setCount(0);

}

//WHAT DO WE RETURN? AN Intersection??
void BVHIntersection(Ray& r, Node& n) {
	if (rayBoxIntersection(r, n.bounds))
	{
		if (n.isLeaf()) {
			for (int i; i < n.count; i++) { 
				int index = i + n.count;
                CoreTri &triangle = BVH::primitives[BVH::indices[index]];
				
				//INTERSECT TRIANGLE
                float intersectDistance = r.calcIntersectDist(triangle);
			}
        } else {
			//intersect left and right
            BVHIntersection(r, BVH::nodes[n.left()]);
            BVHIntersection(r, BVH::nodes[n.right()]);
		}

	}
}


bool rayBoxIntersection(const Ray& r, const AABB& box) {
	// https: //gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
	float3 lb = box.minBounds;
    float3 rt = box.maxBounds;

	float3 dirfrac;
    // r.dir is unit direction vector of ray
    dirfrac.x = 1.0f / r.direction.x;
    dirfrac.y = 1.0f / r.direction.y;
    dirfrac.z = 1.0f / r.direction.z;
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // r.org is origin of ray
    float t1 = (lb.x - r.origin.x) * dirfrac.x;
    float t2 = (rt.x - r.origin.x) * dirfrac.x;
    float t3 = (lb.y - r.origin.y) * dirfrac.y;
    float t4 = (rt.y - r.origin.y) * dirfrac.y;
    float t5 = (lb.z - r.origin.z) * dirfrac.z;
    float t6 = (rt.z - r.origin.z) * dirfrac.z;

    float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0) {
        t = tmax;
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        t = tmax;
        return false;
    }

    t = tmin;
    return true;


}

} // namespace lh2core
