#pragma once
#include "BBox.h"
#include <glm/glm.hpp>
#define uint unsigned int

//here primitive refer to triangle
class Primitive      
{
public:
	Primitive();
	Primitive(const glm::vec3* _vertices, const glm::vec3* _d_vertices, const uint _v0, const uint _v1, const uint _v2);
	/**
	* Get the world space bounding box of the primitive.
	* \return world space bounding box of the primitive
	*/
	BBox get_expand_bbox() const ;
	BBox get_bbox() const;
	__device__ BBox d_get_expand_bbox() const;
	__device__ BBox d_get_bbox() const;

	/**
	* Check if the given point intersects with the primitive, no intersection
	* information is stored
	* \return true if the given point intersects with the primitive,
	false otherwise
	*/
	bool intersect(const glm::vec3& point) const;
	__device__ bool d_intersect(const glm::vec3& point, float &dist, glm::vec3 &normal) const;

	glm::vec3 get_normal() const;
	__device__ glm::vec3 d_get_normal() const;

	
public:
	const glm::vec3* vertices;  
	const glm::vec3* d_vertices;  //for device, ptr to body vertices
	uint v0;
	uint v1;
	uint v2;
};
