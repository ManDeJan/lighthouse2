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

float calculateRawSAH(AABB bounds) { //not including the number of primitves
    float3 box = bounds.maxBounds - bounds.minBounds;
    return (2 * box.x * box.y + 2 * box.y * box.z + 2 * box.z * box.x);
}

AABB calculateBounds(const vector<uint> &indices) {
    float3 minBounds =
        make_float3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    float3 maxBounds =
        make_float3(-numeric_limits<float>::max(), -numeric_limits<float>::max(), -numeric_limits<float>::max());

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

//AABB calculateCentroidBounds(const vector<uint> &indices) {
AABB calculateCentroidBounds(int first, int count) {
    float3 minBounds =
        make_float3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    float3 maxBounds =
        make_float3(-numeric_limits<float>::max(), -numeric_limits<float>::max(), -numeric_limits<float>::max());

    //calculate bounds
    for (int i = 0; i < count; i++) {
        CoreTri &primitive = BVH::primitives[BVH::indices[first + i]];

        float3 primCenter = triangleCenter(primitive);

        minBounds = fminf(minBounds, primCenter);
        maxBounds = fmaxf(maxBounds, primCenter);
    }
    return AABB(minBounds, maxBounds);
}

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
    if (node.isLeaf()) cout << "[" << node.count << "]";
    else {
        cout << "[";
        printBVH(BVH::nodes[node.left()]);
        printBVH(BVH::nodes[node.right()]);
        cout << "]";
    }
}

void Node::subdivide() {
    if (count <= 3) return;

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
    for (size_t i = 0; i < right.size(); i++) { BVH::indices[first() + left.size() + i] = right[i]; }
    BVH::nodes[BVH::nodeIndex + 2] = Node(first() + left.size(), right.size(), rightAABB);

    //setLeftNode and set to parent node
    setLeft(BVH::nodeIndex + 1);
    setChildCount(2);
}

AABB mergeBounds(AABB a, AABB b) {
    float3 min_bounds, max_bounds;
    min_bounds.x = min(a.minBounds.x, b.minBounds.x); 
    min_bounds.y = min(a.minBounds.y, b.minBounds.y); 
    min_bounds.z = min(a.minBounds.z, b.minBounds.z); 
    max_bounds.x = max(a.maxBounds.x, b.maxBounds.x); 
    max_bounds.y = max(a.maxBounds.y, b.maxBounds.y); 
    max_bounds.z = max(a.maxBounds.z, b.maxBounds.z); 
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

    AABB cBounds = calculateCentroidBounds(first(), count);
    float3 dim = cBounds.maxBounds - cBounds.minBounds;

    //Populate bins
    if (dim.x >= dim.y && dim.x >= dim.z) {
        k0 = cBounds.minBounds.x;
        k1 = (nBins * (1 - EPSILON)) / dim.x;

        for (int i = 0; i < count; i++) {
            uint indexi = BVH::indices[first() + i];
            CoreTri prim = BVH::primitives[indexi];
            float primCenter = triangleCenter(prim).x;

            int binId = k1 * (primCenter - k0);
            //print ("binId: ", binId, " k1: ", k1, "primcenter", primCenter, "k0", k0);
            bins[binId].addPrim(indexi);
        }

    } else if (dim.y >= dim.x && dim.y >= dim.z) {
        k0 = cBounds.minBounds.y;
        k1 = (nBins * (1 - EPSILON)) / dim.y;

        for (int i = 0; i < count; i++) {
            uint indexi = BVH::indices[first() + i];
            CoreTri prim = BVH::primitives[indexi];
            float primCenter = triangleCenter(prim).y;

            int binId = k1 * (primCenter - k0);
            bins[binId].addPrim(indexi);
        }
    } else /* z dim is largest */ {
        k0 = cBounds.minBounds.z;
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
        Bin &a = leftBins[i - 1]; //previous aggregated bins
        Bin &b = leftBins[i];     //current bin to be calculated
        Bin &bin = bins[i];       //bin to be added to current bin

        b.primIndices = a.primIndices;
        b.primIndices.insert(b.primIndices.end(), bin.primIndices.begin(), bin.primIndices.end());
        // print("a bounds: min(", a.bounds.minBounds.x, ") b bounds: minx: ", b.bounds.minBounds.x);
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
        Bin &a = rightBins[ii - i + 1]; //previous aggregated bins
        Bin &b = rightBins[ii - i];     //current bin to be calculated
        Bin &bin = bins[ii - i];        //bin to be added to current bin

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
        Bin &left = leftBins[i];
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

    if (!split_worthwile) { return; };

    convertNode(bestLeft.primIndices, bestLeft.bounds, bestRight.primIndices, bestRight.bounds);
}


AABB mergeBoundsVec(vector<Node> nodes) {
    AABB result = nodes[0].bounds;
    for (int i = 1; i < nodes.size(); i++) { mergeBounds(result, nodes[i].bounds); }
    return result;
}

void BVH::convertBVH4() {
    
	vector<Node> nodes2 = nodes;
    
	nodes.clear();
    nodes.resize(nodes2.size());
    nodes[0] = nodes2[0];
    root = &nodes[0];
    nodeIndex = 0;

	transformBVH4Node(*root, nodes2);	

}

bool sortAABB(Node a, Node b) {
	return calculateRawSAH(a.bounds) >= calculateRawSAH(b.bounds);
}

void BVH::transformBVH4Node(Node &node, vector<Node> &nodes2) {
	
	if (node.isLeaf()) return;

	Node lChild = nodes2[node.left()];
    Node rChild = nodes2[node.right()];
	
	vector<Node> newChilds;

	if (!lChild.isLeaf()) { 
		newChilds.push_back(nodes2[lChild.left()]);
        newChilds.push_back(nodes2[lChild.right()]);
    } else {
        newChilds.push_back(lChild);
	}
    if (!rChild.isLeaf()) {
        newChilds.push_back(nodes2[rChild.left()]);
        newChilds.push_back(nodes2[rChild.right()]);
    } else {
        newChilds.push_back(rChild);
    }
	sort(newChilds.begin(), newChilds.end(), sortAABB);
	
	//node.setBounds(mergeBoundsVec(newChilds));
    node.setLeft(nodeIndex + 1);
    node.setChildCount(newChilds.size());
	
    nodeIndex += newChilds.size();

	for (int i = 0; i < newChilds.size(); i++) { 
		nodes[node.left() + i] = newChilds[i];
	}

	for (int i = 0; i < newChilds.size(); i++) 
	{ 
		transformBVH4Node(nodes[node.left() + i], nodes2); 
	}

}


int calcSurfaceArea(AABB a) {
    return (a.maxBounds.x - a.minBounds.x) * (a.maxBounds.y - a.minBounds.y) * (a.maxBounds.z - a.minBounds.z);
}


} // namespace lh2core
