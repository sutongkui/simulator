#include<map>

#include "Ray.h"
#include <glm/glm.hpp>

#define MOLLER_TRUMBORE

bool rayTriangleIntersect(
	Ray ray,
	const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
	float& t, float& u, float& v)
{
#ifdef MOLLER_TRUMBORE 
	glm::vec3 v0v1 = v1 - v0;
	glm::vec3 v0v2 = v2 - v0;
	glm::vec3 pvec = glm::cross(ray.dir, v0v2);
	float det = glm::dot(v0v1, pvec);
#ifdef CULLING 
	// if the determinant is negative the triangle is backfacing
	// if the determinant is close to 0, the ray misses the triangle
	if (det < kEpsilon) return false;
#else 
	// ray and triangle are parallel if det is close to 0
	if (fabs(det) < FLT_EPSILON) return false;
#endif 
	float invDet = 1 / det;

	glm::vec3 tvec = ray.origin - v0;
	u = glm::dot(tvec, pvec) * invDet;
	if (u < 0 || u > 1) return false;

	glm::vec3 qvec = glm::cross(tvec, v0v1);
	v = glm::dot(ray.dir, qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = glm::dot(v0v2, qvec) * invDet;

	return true;
#else 

#endif 
}


int find_first_intersect_triangle(Ray ray, const Mesh* mesh)
{
	auto num_face = mesh->faces.size();
	map<float, int> isect_points;

	for (auto i = 0; i < num_face; i++)
	{
		auto idx = mesh->faces[i].vertex_index;

		float t = 0,u,v;
		glm::vec3 ver[3];
		for (int i = 0; i < 3; i++)
		{
			ver[i] = glm::vec3(mesh->vertices[idx[i]]);
		}

		auto is_isect = rayTriangleIntersect(ray,ver[0],ver[1],ver[2], t,u,v);
		if (is_isect)
		{
			isect_points[t] = i;
		}
	}

	if (isect_points.empty())
	{
		return -1;
	}
	else
	{
		return isect_points.begin()->second;
	}
	
}
