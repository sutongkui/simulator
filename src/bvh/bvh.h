#pragma once
#include <vector> 

#include "bbox.h"
#include "primitive.h"
#include "cudaBRT.h"
#include "../ObjLoader.h"
#include "../Mesh.h"


using MortonCode = unsigned int;

/**
* Bounding Volume Hierarchy for fast point-objects intersection.
* Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
* all the primitives it was built from. Therefore once a BVHAccel Aggregate
* is created, the original input primitives can be ignored from the scene
* during point-objects intersection tests as they are contained in the aggregate.
*/
class BVHAccel{
public:

	BVHAccel() { }
	/**
	* Parameterized Constructor.
	* Create BVH from a list of primitives. Note that the BVHAccel Aggregate
	* stores pointers to the primitives and thus the primitives need be kept
	* in memory for the aggregate to function properly.
	* \param primitives primitives to build from
	* \param max_leaf_size maximum number of primitives to be stored in leaves
	*/
	BVHAccel(Mesh& body, size_t max_leaf_size = 4);

	/**
	* Destructor.
	* The destructor only destroys the Aggregate itself, the primitives that
	* it contains are left untouched.
	*/
	~BVHAccel();

#ifdef _DEBUG
	BRTreeNode* get_root() const;
	BRTreeNode* get_left_child(BRTreeNode* node) const;
	BRTreeNode* get_right_child(BRTreeNode* node) const;
	bool is_leaf(BRTreeNode* node) const;
	bool intersect(const glm::vec3 point, int& idx) const;
	bool check_overlap(const glm::vec3 point, BRTreeNode* node) const;

	//显示包围盒之前需要调用，完成数据从GPU到CPU的拷贝
	void copy_data_gpu_to_cpu();  //for test
	void print(BRTreeNode* root, int depth, const int max_depth);
	void draw(BRTreeNode* root);
	void access(BRTreeNode* root, vector<BRTreeNode*>& bad_bode);
#endif

private:

	//functions for morton code based BVH construction algorithm
	unsigned int expandBits(unsigned int v);
	unsigned int morton3D(float x, float y, float z);
	unsigned int morton3D(glm::vec3 pos);
	void compute_bbox_and_morton();
	BBox computet_root_bbox(Primitive* d_tem_primitives);    // get root AABB size
	void init();
	void build();
	void init_primitives(Mesh& body);

	BRTreeNode* get_leaf_nodes();
	BRTreeNode* get_internal_nodes();

private:

	// 辅助计算
	vector<BBox> _bboxes;
	vector<BBox> _sorted_bboxes;

	vector<MortonCode> _morton_codes;
	vector<MortonCode> _sorted_morton_codes;

	vector<Primitive> _primitives;
	vector<Primitive> _sorted_primitives;

	BBox* d_bboxes;
	MortonCode* d_sorted_morton_code;

	vector<glm::vec3> obj_vertices;  
	glm::vec3* d_obj_vertices;

	int numInternalNode;
	int numLeafNode;

#ifdef _DEBUG
	BRTreeNode* h_leaf_nodes;
	BRTreeNode* h_internal_nodes;
#endif

public:
	// external interface
	D_BVH* d_bvh;
	
};
