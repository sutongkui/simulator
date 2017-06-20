#include "primitive.h"

Primitive::Primitive(): vertices(nullptr), d_vertices(nullptr) {}
Primitive::Primitive(const glm::vec3* _vertices, const glm::vec3* _d_vertices, const uint _v0, const uint _v1, const uint _v2)
	:vertices(_vertices), d_vertices(_d_vertices), v0(_v0), v1(_v1), v2(_v2) {}

BBox Primitive::get_bbox() const
{
	BBox bbox = (vertices[v0]);
	bbox.expand(vertices[v1]);
	bbox.expand(vertices[v2]);

	return bbox;
}

BBox Primitive::get_expand_bbox() const
{
	BBox bbox = get_bbox();
	
	//沿着法线方向适当拓展或收缩三角面片,后期改为点的各自法线方向
	float depth = 0.01;
	glm::vec3 n = get_normal();
	bbox.expand(vertices[v0] - depth*n);
	bbox.expand(vertices[v1] - depth*n);
	bbox.expand(vertices[v2] - depth*n);

	return bbox;
}

BBox  Primitive::d_get_bbox() const
{
	BBox bbox(d_vertices[v0]);
	bbox.expand(d_vertices[v1]);
	bbox.expand(d_vertices[v2]);

	return bbox;
}

BBox  Primitive::d_get_expand_bbox() const
{
	BBox bbox = d_get_bbox();
	
	//沿着法线方向适当拓展或收缩三角面片,后期改为点的各自法线方向
	float depth = 0.01;
	glm::vec3 n = d_get_normal();
	bbox.expand(d_vertices[v0] - depth * n);
	bbox.expand(d_vertices[v1] - depth * n);
	bbox.expand(d_vertices[v2] - depth * n);

	return bbox;
}

bool Primitive::intersect(const glm::vec3& point) const
{
	//use normal or barycentric coordinates
	glm::vec3 side1, side2, normalface;
	side1 = vertices[v1] - vertices[v0];
	side2 = vertices[v2] - vertices[v0];
	normalface = glm::cross(side1, side2);
	

	glm::vec3 tem = point - vertices[v0];
	
	if (glm::dot(tem, normalface) > 0)
		return true;

	return false;
}

bool Primitive::d_intersect(const glm::vec3& point, float &dist, glm::vec3 &normal) const
{
	//use normal or barycentric coordinates
	glm::vec3 side1, side2, normalface;
	side1 = d_vertices[v1] - d_vertices[v0];
	side2 = d_vertices[v2] - d_vertices[v0];
	normalface = glm::cross(side1, side2);
	normal = glm::normalize(normalface);

	glm::vec3 tem = point - d_vertices[v0];
	dist = glm::dot(tem, normal);
	if (dist > 0)
		return false;

	return true;
}

glm::vec3 Primitive::get_normal() const
{
	glm::vec3 side1, side2, normalface;
	side1 = vertices[v1] - vertices[v0];
	side2 = vertices[v2] - vertices[v0];
	normalface = glm::cross(side1, side2);
	return glm::normalize(normalface);
}

glm::vec3 Primitive::d_get_normal() const
{
	glm::vec3 side1, side2, normalface;
	side1 = d_vertices[v1] - d_vertices[v0];
	side2 = d_vertices[v2] - d_vertices[v0];
	normalface = glm::cross(side1, side2);
	return glm::normalize(normalface);
}