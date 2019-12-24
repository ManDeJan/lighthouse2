#include "BVH.h"

#include "rendercore.h"

namespace lh2core {

Node *BVH::root;
vector<Node> BVH::nodes;
size_t BVH::nodeIndex;
vector<uint> BVH::indices;
vector<CoreTri> BVH::primitives;

float3 triangleCenter(CoreTri &tri) {
    return (tri.vertex0 + tri.vertex1 + tri.vertex2) / 3.0f;
}

float calculateRawSAH(AABB bounds) {//not including the number of primitves
    float3 box = bounds.maxBounds - bounds.minBounds;
    //return BVH::indices.size() * (2 * box.x * box.y + 2 * box.y * box.z + 2 * box.z * box.x);
    return (2 * box.x * box.y + 2 * box.y * box.z + 2 * box.z * box.x);
}


AABB calculateBounds(const vector<uint> &indices) {
    float3 minBounds =
        make_float3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    float3 maxBounds =
        make_float3(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());

    // float3 minBounds = make_float3(0, 0, 0);
    // float3 maxBounds = make_float3(0, 0, 0);

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
    print("Constructing BVH, size ", N);
    indices.resize(N);
    for (size_t i = 0; i < N; i++) indices[i] = i;
    // allocate BVH root node
    nodes.clear();
    nodes.resize(N * 2 - 1);
    root = &nodes[0];
    nodeIndex = 0;
    print("Indices Build BVH");
    // subdivide root node
    root->setFirst(0);
    root->setCount(N);
    root->setBounds(calculateBounds(indices));
    root->subdivide();
}

void BVH::setMesh(Mesh &mesh) {
    auto a = vector<CoreTri>(mesh.triangles, mesh.triangles + mesh.vcount / 3);
    primitives.insert(primitives.end(), a.begin(), a.end());
}

int split_count = 0;

void printBVH(Node &node) {
	if (node.isLeaf())
		cout << "[" << node.count << "]";
	else {
		cout << "[";
		printBVH(BVH::nodes[node.left()]);
		printBVH(BVH::nodes[node.right()]);
		cout << "]";
	}
}

void Node::subdivide() {

    // print("\n\nsplit: ", ++split_count);
    // printBVH(*BVH::root);
    // print();
    // int i = 0;
    // for (auto &node : BVH::nodes) {
    //     if (node.isLeaf()) {
    //         print(i++, " BLAD! : ", node.count, " ", node.first());
    //     } else {
    //         print(i++, " TAK!  : ", node.left());
    //     }
    // }

    if (count <= 4) return;

    binnedPartition();
    // print("noot indeks", BVH::nodeIndex);
    BVH::nodeIndex += 2;
    BVH::nodes[left()].subdivide();
    BVH::nodes[right()].subdivide();
    
}


void Node::partition() {
    float bestSplitCost = numeric_limits<float>::max();
    vector<uint> bestSplitLeft, bestSplitRight;
    AABB bestBSL, bestBSR;

    for (int i = 0; i < count; i++) {
        // if (!(i % 1000)) print("Partitioning: ", i);
        CoreTri primSplit = BVH::primitives[BVH::indices[first() + i]];
        float3 primSplitCenter = triangleCenter(primSplit);

        vector<uint> splitLeft, splitRight;
        AABB boundsSL, boundsSR;

        // X
        //split left and right on X
        for (int j = 0; j < count; j++) {
            uint indexj = BVH::indices[first() + j];
            CoreTri prim = BVH::primitives[indexj];
            float3 primCenter = triangleCenter(prim);

            if (primCenter.x < primSplitCenter.x) splitLeft.push_back(indexj);
            else
                splitRight.push_back(indexj);
        }

        //calculate cost on X
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        float cost = splitLeft.size() * calculateRawSAH(boundsSL) + splitRight.size() * calculateRawSAH(boundsSR);
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
                splitRight.push_back(indexj);
        }

        //calculate cost on Y
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        cost = splitLeft.size() * calculateRawSAH(boundsSL) + splitRight.size() * calculateRawSAH(boundsSR);
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
                splitRight.push_back(indexj);
        }

        //calculate cost on Z
        boundsSL = calculateBounds(splitLeft);
        boundsSR = calculateBounds(splitRight);
        cost = splitLeft.size() * calculateRawSAH(boundsSL) + splitRight.size() * calculateRawSAH(boundsSR);
        if (cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitLeft = splitLeft;
            bestSplitRight = splitRight;
            bestBSL = boundsSL;
            bestBSR = boundsSR;
        }
    }

	convertNode(bestSplitLeft, bestBSL, bestSplitRight, bestBSR);
}

void Node::convertNode(vector<uint> left, AABB leftAABB, vector<uint> right, AABB rightAABB) {
    //Create left node
    for (size_t i = 0; i < left.size(); i++) BVH::indices[first() + i] = left[i];
    BVH::nodes[BVH::nodeIndex + 1] = Node(first(), left.size(), leftAABB);

    //create right node
    for (size_t i = 0; i < right.size(); i++) {
        BVH::indices[first() + left.size() + i] = right[i];
    }
    BVH::nodes[BVH::nodeIndex + 2] = Node(first() + left.size(), right.size(), rightAABB);

    //setLeftNode and set to parent node
    setLeft(BVH::nodeIndex + 1);
    setCount(0);
}

AABB mergeBounds(AABB a, AABB b) {
    float3 min_bounds, max_bounds;
    min_bounds.x = min(a.minBounds.x, b.minBounds.x); // ? a.minBounds.x : b.minBounds.x;
    min_bounds.y = min(a.minBounds.y, b.minBounds.y); // ? a.minBounds.y : b.minBounds.y;
    min_bounds.z = min(a.minBounds.z, b.minBounds.z); // ? a.minBounds.z : b.minBounds.z;
    max_bounds.x = max(a.maxBounds.x, b.maxBounds.x); // ? a.maxBounds.x : b.maxBounds.x;
    max_bounds.y = max(a.maxBounds.y, b.maxBounds.y); // ? a.maxBounds.y : b.maxBounds.y;
    max_bounds.z = max(a.maxBounds.z, b.maxBounds.z); // ? a.maxBounds.z : b.maxBounds.z;
    return (AABB(min_bounds, max_bounds));
}


void Bin::evaluateBounds() {
    bounds = calculateBounds(primIndices);
}
AABB Bin::evaluateGetBounds() {
    return bounds = calculateBounds(primIndices);
}

void Node::binnedPartition() {
	//16 bins along widest axis
    constexpr int nBins = 16;
    Bin bins[nBins];
    float k1, k0;

	float3 dim = bounds.maxBounds - bounds.minBounds;
	
	//Populate bins
	if (dim.x >= dim.y && dim.x >= dim.z) {
        k0 = bounds.minBounds.x;
        k1 = (nBins * (1 - EPSILON)) / dim.x;

		for (int i = 0; i < count; i++) {
            uint indexi = BVH::indices[first() + i];
            CoreTri prim = BVH::primitives[indexi];
            float primCenter = triangleCenter(prim).x;

			int binId = k1 * (primCenter - k0);
            bins[binId].addPrim(indexi);
		}

	} else if (dim.y >= dim.x && dim.y >= dim.z) {
        k0 = bounds.minBounds.y;
        k1 = (nBins * (1 - EPSILON)) / dim.y;

        for (int i = 0; i < count; i++) {
            uint indexi = BVH::indices[first() + i];
            CoreTri prim = BVH::primitives[indexi];
            float primCenter = triangleCenter(prim).y;

            int binId = k1 * (primCenter - k0);
            bins[binId].addPrim(indexi);
        }
	} else /* z dim is largest */ {
        k0 = bounds.minBounds.z;
        k1 = (nBins * (1 - EPSILON)) / dim.z;

        for (int i = 0; i < count; i++) {
            uint indexi = BVH::indices[first() + i];
            CoreTri prim = BVH::primitives[indexi];
            float primCenter = triangleCenter(prim).z;

            int binId = k1 * (primCenter - k0);
            bins[binId].addPrim(indexi);
        }
	}

	Bin leftBins[nBins]; //aggregation of bins
    
	bins[0].evaluateBounds();
	leftBins[0] = bins[0];
    leftBins[0].cost = calculateRawSAH(leftBins[0].bounds) * leftBins[0].count;

	//evaluateBounds for each bin and sweep from left
    for (int i = 1; i < nBins; i++) { 
        Bin &a = leftBins[i - 1];	//previous aggregated bins
        Bin &b = leftBins[i];		//current bin to be calculated
        Bin &bin = bins[i];			//bin to be added to current bin

		b.primIndices = a.primIndices;
        b.primIndices.insert(b.primIndices.end(), bin.primIndices.begin(), bin.primIndices.end());
        b.bounds = mergeBounds(a.bounds, bin.evaluateGetBounds());
        b.count = a.count + bin.count;
        b.cost = calculateRawSAH(b.bounds) * b.count;
	}

    
	Bin rightBins[nBins]; //aggregation of bins
    int ii = nBins - 1;
    bins[ii].evaluateBounds();
    rightBins[ii] = bins[ii];
    rightBins[ii].cost = calculateRawSAH(rightBins[ii].bounds) * rightBins[ii].count;

    //evaluateBounds for each bin and sweep from right
    for (int i = 1; i < nBins; i++) {
        Bin &a = rightBins[ii - i + 1];	//previous aggregated bins
        Bin &b = rightBins[ii - i];		//current bin to be calculated
        Bin &bin = bins[ii - i];		//bin to be added to current bin

        b.primIndices = a.primIndices;
        b.primIndices.insert(b.primIndices.end(), bin.primIndices.begin(), bin.primIndices.end());
        b.bounds = mergeBounds(a.bounds, bin.evaluateGetBounds());
        b.count = a.count + bin.count;
        b.cost = calculateRawSAH(b.bounds) * b.count;
    }

    // for (auto blin : leftBins) {
    //     print("Leftbin: ", blin.count);
    // }
    // for (auto blin : rightBins) {
    //     print("Rightbin: ", blin.count);
    // }

	//find optimal split
	float optimalCost = numeric_limits<float>::max();
    Bin bestLeft;
	Bin bestRight;
    bool split_worthwile = false;

	for (int i = 0; i < nBins - 1; i++) { 
		Bin &left  = leftBins[i];
        Bin &right = rightBins[i + 1];
		float totalCost = left.cost + right.cost;
        // print("left count: ", left.count, " right count: ", right.count);
        // print("left cost : ", left.cost , " right cost : ", right.cost );
		if (totalCost < optimalCost) {
            split_worthwile = true;
			optimalCost = totalCost;
            bestLeft = left;
            bestRight = right;
		}
	}

    if (!split_worthwile) return;
    // print("bestleft count: ", bestLeft.count, " bestright count: ", bestRight.count);

    // if (bestLeft.count + bestRight.count > BVH::indices.size()) {
    //     for (auto i : bestLeft.primIndices) {
    //         for (auto j : bestRight.primIndices) {
    //             if (i == j) print("double: ", i);
    //         }
    //     }
    //     print("CHAOS HELP ", bestLeft.count + bestRight.count, " ", BVH::indices.size());
    // } else {
    //     print("yay? ", bestLeft.count + bestRight.count, " ", BVH::indices.size());
    // }
    
    convertNode(bestLeft.primIndices, bestLeft.bounds, bestRight.primIndices, bestRight.bounds);
}

// bool rayBoxIntersection(const Ray& r, const AABB& box) {
// 	// https: //gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
// 	float3 lb = box.minBounds;
//     float3 rt = box.maxBounds;

//     float3 dirfrac;
//     // r.dir is unit direction vector of ray
//     dirfrac.x = 1.0f / r.direction.x;
//     dirfrac.y = 1.0f / r.direction.y;
//     dirfrac.z = 1.0f / r.direction.z;
//     // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
//     // r.org is origin of ray
//     float t1 = (lb.x - r.origin.x) * dirfrac.x;
//     float t2 = (rt.x - r.origin.x) * dirfrac.x;
//     float t3 = (lb.y - r.origin.y) * dirfrac.y;
//     float t4 = (rt.y - r.origin.y) * dirfrac.y;
//     float t5 = (lb.z - r.origin.z) * dirfrac.z;
//     float t6 = (rt.z - r.origin.z) * dirfrac.z;

//     float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
//     float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

//     // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
//     if (tmax < 0) {
//         //outcommented t, I don't think we need to return t for the bounding box right?
//         //t = tmax;
//         return false;
//     }

//     // if tmin > tmax, ray doesn't intersect AABB
//     if (tmin > tmax) {
//         //t = tmax;
//         return false;
//     }

//     //t = tmin;
//     return true;
// }

// //WHAT DO WE RETURN? AN Intersection??
// Intersection BVHIntersection(Ray &r, Node &n) {
//     if (rayBoxIntersection(r, n.bounds)) {
//         if (n.isLeaf()) {
//             Intersection j;
//             for (size_t i = 0; i < n.count; i++) {
//                 size_t index = i + n.count;
//                 CoreTri &triangle = BVH::primitives[BVH::indices[index]];

//                 //INTERSECT TRIANGLE
//                 float t = r.calcIntersectDist(triangle);
//                 if (t < j.distance && t > 0) {
//                     j.distance = t;
//                     j.triangle = &triangle;
//                     j.location = r.origin + r.direction * j.distance;
//                 }
//             }
//             return j;
//         } else {
//             //intersect left and right
//             BVHIntersection(r, BVH::nodes[n.left()]);
//             BVHIntersection(r, BVH::nodes[n.right()]);
//         }
//     }
// }
} // namespace lh2core
