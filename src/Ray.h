#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "Mesh.h"


struct Ray
{
	Ray() {}
	Ray(glm::vec3 _origin, glm::vec3 _dir) :origin(_origin), dir(_dir) {}

	glm::vec3 origin;
	glm::vec3 dir; // normalize
};

// return the index of the triangle
extern int find_first_intersect_triangle(Ray ray, const Mesh* mesh);