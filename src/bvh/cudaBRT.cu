#include "cudaBRT.h"
#include <iostream>

/**
* delta operator measures the common prefix of two morton_code
* if j is not in the range of the sorted_morton_code,
* delta operator returns -1.
*/
__device__ int delta(int i, int j, unsigned int* sorted_morton_code, int length)
{
	if (j<0 || j >= length)
	{
		return -1;
	}
	else
	{
		return __clz(sorted_morton_code[i] ^ sorted_morton_code[j]);
	}
}

/**
* determine the range of an internal node
*/
__device__ int2 determineRange(unsigned int* sorted_morton_code, int numInternalNode, int i)
{
	int size = numInternalNode + 1;
	int d = delta(i, i + 1, sorted_morton_code, size) - delta(i, i - 1, sorted_morton_code, size);
	d = d > 0 ? 1 : -1;

	//compute the upper bound for the length of the range
	int delta_min = delta(i, i - d, sorted_morton_code, size);
	int lmax = 2;
	while (delta(i, i + lmax*d, sorted_morton_code, size)>delta_min)
	{
		lmax = lmax * 2;
	}

	//find the other end using binary search
	int l = 0;
	for (int t = lmax / 2; t >= 1; t /= 2)
	{
		if (delta(i, i + (l + t)*d, sorted_morton_code, size)>delta_min)
		{
			l = l + t;
		}
	}
	int j = i + l*d;

	int2 range;
	if (i <= j) { range.x = i; range.y = j; }
	else { range.x = j; range.y = i; }
	return range;
}

/**
* to judge if two values differes
* in bit position n
*/
__device__ bool is_diff_at_bit(unsigned int val1, unsigned int val2, int n)
{
	return val1 >> (31 - n) != val2 >> (31 - n);
}

/**
* find the best split position for an internal node
*/
__device__ int findSplit(unsigned int* sorted_morton_code, int start, int last)
{
	//return -1 if there is only 
	//one primitive under this node.
	if (start == last)
	{
		return -1;
	}
	else
	{
		int common_prefix = __clz(sorted_morton_code[start] ^ sorted_morton_code[last]);

		//handle duplicated morton code separately
		if (common_prefix == 32)
		{
			return (start + last) / 2;
		}

		// Use binary search to find where the next bit differs.
		// Specifically, we are looking for the highest object that
		// shares more than commonPrefix bits with the first one.

		int split = start; // initial guess
		int step = last - start;
		do
		{
			step = (step + 1) >> 1; // exponential decrease
			int newSplit = split + step; // proposed new position

			if (newSplit < last)
			{
				bool is_diff = is_diff_at_bit(sorted_morton_code[start],
					sorted_morton_code[newSplit],
					common_prefix);
				if (!is_diff)
				{
					split = newSplit; // accept proposal
				}
			}
		} while (step > 1);

		return split;
	}
}

//FOR BR-TREE CONSTRUCTION
//TODO: implement internal node processing routine
//TODO: handle duplicated morton codes as special case (using their position i,j as fallback)

//FOR BVH CONSTRUCTION
//TODO: implement AABB construction process by go back from the tree node to the root
//TODO: convert BR-TREE BACK TO BVH
//TODO: debug
__global__  void processInternalNode(unsigned int* sorted_morton_code, int numInternalNode,
	BRTreeNode* leafNodes,
	BRTreeNode* internalNodes)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numInternalNode) return;


	// Find out which range of objects the node corresponds to.
	int2 range = determineRange(sorted_morton_code, numInternalNode, idx);
	int first = range.x;
	int last = range.y;

	// Determine where to split the range.
	int split = findSplit(sorted_morton_code, first, last);

	if (split == -1) return;

	// Select childA.
	BRTreeNode* childA;
	bool isChildALeaf = false;
	if (split == first) {
		childA = &(leafNodes[split]);
		isChildALeaf = true;
	}
	else childA = &(internalNodes[split]);

	// Select childB.
	BRTreeNode* childB;
	bool isChildBLeaf = false;
	if (split + 1 == last) {
		childB = &(leafNodes[split + 1]);
		isChildBLeaf = true;
	}
	else childB = &(internalNodes[split + 1]);

	// Record parent-child relationships.
	internalNodes[idx].setChildA(split, isChildALeaf);
	internalNodes[idx].setChildB(split + 1, isChildBLeaf);
	childA->setParent(idx);
	childB->setParent(idx);
}

/**
* construct bounding boxes from leaf up to root
*/
__global__  void calculateBoudingBox(BBox* d_bboxes, int numLeafNode,
	BRTreeNode* leafNodes, BRTreeNode* internalNodes)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= numLeafNode) return;



	//handle leaf first
	BRTreeNode* node = &leafNodes[idx];
	node->bbox = d_bboxes[idx];


	//terminate if it is root node
	bool is_null = false;
	int parentIdx = node->getParent(is_null);
	//if (is_null) return;
	node = &internalNodes[parentIdx];

	int initial_val = atomicInc(&node->counter, 1);
	while (1)
	{
		if (initial_val == 0) return; //terminate the first accesing thread

									  //calculate bounding box by merging two children's bounding box
		bool is_leaf = false;
		int childAIdx = node->getChildA(is_leaf, is_null);
		if (is_leaf) node->bbox.expand(leafNodes[childAIdx].bbox);
		else node->bbox.expand(internalNodes[childAIdx].bbox);

		int childBIdx = node->getChildB(is_leaf, is_null);
		if (is_leaf) node->bbox.expand(leafNodes[childBIdx].bbox);
		else node->bbox.expand(internalNodes[childBIdx].bbox);

		//terminate if it is root node
		parentIdx = node->getParent(is_null);
		if (is_null) return;
		node = &internalNodes[parentIdx];
		initial_val = atomicInc(&node->counter, 1);
	}
}



D_BVH::D_BVH() :
	d_primitives(nullptr),
	d_leaf_nodes(nullptr),
	d_internal_nodes(nullptr)
{

}

D_BVH::D_BVH(Primitive* _d_primitives,
	BRTreeNode* _d_leaf_nodes,
	BRTreeNode* _d_internal_nodes) :

	d_primitives(_d_primitives),
	d_leaf_nodes(_d_leaf_nodes),
	d_internal_nodes(_d_internal_nodes)
{

}

D_BVH::~D_BVH()
{
	
}
void D_BVH::free_memory()
{
	cudaFree(d_primitives);
	cudaFree(d_leaf_nodes);
	cudaFree(d_internal_nodes);
}
 bool D_BVH::intersect(const glm::vec3 point, int& idx)
{
	// Allocate traversal stack from thread-local memory,
	// and push NULL to indicate that there are no postponed nodes.
	BRTreeNode* stack[64];
	BRTreeNode** stackPtr = stack;
	*stackPtr++ = NULL; // push

						// Traverse nodes starting from the root.
	BRTreeNode* node = get_root();
	do
	{
		// Check each child node for overlap.
		BRTreeNode* childA = get_left_child(node);
		BRTreeNode* childB = get_right_child(node);
		bool overlapL = check_overlap(point, childA);
		bool overlapR = check_overlap(point, childB);

		// Query overlaps a leaf node => report collision with the first collision.
		if (overlapL && is_leaf(childA))
		{
			idx = childA->getIdx();       //is a leaf, and we can get it through primitive[idx]
			return true;
		}

		if (overlapR && is_leaf(childB))
		{
			idx = childB->getIdx();
			return true;
		}

		// Query overlaps an internal node => traverse.
		bool traverseL = (overlapL && !is_leaf(childA));
		bool traverseR = (overlapR && !is_leaf(childB));

		if (!traverseL && !traverseR)
			node = *--stackPtr; // pop
		else
		{
			node = (traverseL) ? childA : childB;
			if (traverseL && traverseR)
				*stackPtr++ = childB; // push
		}
	} while (node != NULL);

	return false;
}
 bool  D_BVH::primitive_intersect(unsigned int idx, const glm::vec3 pos, float& dist, glm::vec3& normal)
 {
	 return d_primitives[idx].d_intersect(pos, dist, normal);
 }
 BRTreeNode*  D_BVH::get_root()
{
	return &d_internal_nodes[0];
}
BRTreeNode*  D_BVH::get_left_child(BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildA(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &d_leaf_nodes[child_idx];
		}
		else
		{
			return &d_internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}
 BRTreeNode*  D_BVH::get_right_child(BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildB(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &d_leaf_nodes[child_idx];
		}
		else
		{
			return &d_internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}
bool  D_BVH::is_leaf(BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null_a = false;
	bool is_null_b = false;
	node->getChildA(is_leaf, is_null_a);
	node->getChildB(is_leaf, is_null_b);

	if (is_null_a && is_null_b)
		return true;
	return false;
}
bool  D_BVH::check_overlap(const glm::vec3 point, BRTreeNode* node)
{
	return node->bbox.intersect(point);
}