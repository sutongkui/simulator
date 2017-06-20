#pragma once
#include <cuda_runtime.h>
#include <glm/glm.hpp>
#include <iostream>

struct BBox
{
	glm::vec3 max;		///< max corner of the bounding box
	glm::vec3 min;	    ///< min corner of the bounding box
	glm::vec3 extent;    ///< extent of the bounding box (min -> max)

	 /**
	 * Constructor.
	 * The default constructor creates a new bounding box which contains no
	 * points.
	 */
	__host__ __device__
	BBox() {
		max = glm::vec3(-100.0, -100.0, -100.0);
		min = glm::vec3(100.0, 100.0, 100.0);
		extent = max - min;
	}

	/**
	* Constructor.
	* Creates a bounding box that includes a single point.
	*/
	__host__ __device__
	BBox(const glm::vec3& p) : min(p), max(p) { extent = max - min; }

	/**
	* Constructor.
	* Creates a bounding box with given bounds.
	* \param min the min corner
	* \param max the max corner
	*/
	__host__ __device__
	BBox(const glm::vec3& min, const glm::vec3& max) :
		min(min), max(max) {
		extent = max - min;
	}

	/**
	* Constructor.
	* Creates a bounding box with given bounds (component wise).
	*/
	__host__ __device__
	BBox(const double minX, const double minY, const double minZ,
		const double maxX, const double maxY, const double maxZ) {
		min = glm::vec3(minX, minY, minZ);
		max = glm::vec3(maxX, maxY, maxZ);
		extent = max - min;
	}

	/**
	* Expand the bounding box to include another (union).
	* If the given bounding box is contained within *this*, nothing happens.
	* Otherwise *this* is expanded to the minimum volume that contains the
	* given input.
	* \param bbox the bounding box to be included
	*/
	__host__ __device__
		void expand(const BBox& bbox) {
		min.x = fmin(min.x, bbox.min.x);
		min.y = fmin(min.y, bbox.min.y);
		min.z = fmin(min.z, bbox.min.z);
		max.x = fmax(max.x, bbox.max.x);
		max.y = fmax(max.y, bbox.max.y);
		max.z = fmax(max.z, bbox.max.z);
		extent.x = max.x - min.x;
		extent.y = max.y - min.y;
		extent.z = max.z - min.z;
	}

	/**
	* Intersects point with bounding box, does not store shading information.
	Checking if a point is inside an AABB is pretty simple ¡ª we just need to check whether the point's coordinates fall inside the AABB; 
	considering each axis separately. If we assume that Px, Py and Pz are the point's coordinates, 
	and BminX¨CBmaxX, BminY¨CBmaxY, and BminZ¨CBmaxZ are the ranges of each exis of the AABB, 
	we can calculate whether a collision has occured between the two using the following formula:

	f(P,B)=(Px>=BminX¡ÄPx<=BmaxX)¡Ä(Py>=BminY¡ÄPy<=BmaxY)¡Ä(Pz>=BminZ¡ÄPz<=BmaxZ)
	*/
	__host__ __device__
	bool intersect(const glm::vec3& point) const
	{
		return (point.x >= min.x && point.x <= max.x) &&
			(point.y >= min.y && point.y <= max.y) &&
			(point.z >= min.z && point.z <= max.z);
	}

	/**
	* Draw box wireframe with OpenGL.
	*/
	void draw() const;
	void print() const;
	__host__ __device__
	glm::vec3 centroid() const {
		glm::vec3 sum = min + max;
		sum /= 2;
		return sum;
	}
	/**
	* Calculate and return an object's
	* normalized position in the unit
	* cube defined by this BBox. if the
	* object is not inside of the BBox, its
	* position will be clamped into the BBox.
	*
	* \param pos the position to be evaluated
	* \return the normalized position in the unit
	* cube, with x,y,z ranging from [0,1]
	*/
	__host__ __device__
	glm::vec3 getUnitcubePosOf(glm::vec3 pos)
	{
		glm::vec3 o2pos = pos - min;
		if (glm::length(extent))
		{
			glm::vec3 normalized_pos = o2pos / extent;
			return normalized_pos;
		}
		else
		{
			return glm::vec3();
		}
	}

};