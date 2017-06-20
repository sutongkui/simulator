
#include "./bvh/bvh.h"
#include "spring.h"
#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <glm/glm.hpp>

#include "sim_parameter.h"


//physics parameter
__constant__ double spring_structure = 100.0;
__constant__ double spring_bend = 2.0;
__constant__ float damp = -0.02f; 
__constant__ float mass = 0.3;
__constant__ float dt = 1 / 40.0f;

__constant__ float gravit_x = 0.0f;   // in y dir
__constant__ float gravit_y = -0.00981f;   // in y dir
__constant__ float gravit_z = 0.0f;   // in y dir


__device__ void collision_response_projection(D_BVH bvh,
	glm::vec3& force, glm::vec3& pos, glm::vec3& pos_old,
	int idx, glm::vec3* collision_force)
{
	int idx_pri;
	bool inter = bvh.intersect(pos, idx_pri);
	if (inter)
	{
		float dist;
		glm::vec3 normal;
		if (bvh.primitive_intersect(idx_pri, pos, dist, normal))  // check the point inside the primitive or not
		{
			float k = 1.0;
			dist = k*glm::abs(dist);    // //collision response with penalty force
			pos += dist*normal;
			pos_old = pos;

			collision_force[idx] = normal;
		}
		else
			collision_force[idx] = glm::vec3(0.0);

	}
	else
		collision_force[idx] = glm::vec3(0.0);

}

__device__ glm::vec3 get_spring_force(int index, glm::vec3* g_pos_in, glm::vec3* g_pos_old_in, glm::vec3* const_pos,
										s_spring* neigh,unsigned int NUM_NEIGH,
									  glm::vec3 pos,glm::vec3 vel,float k_spring)
{
	glm::vec3 force(0.0);
	int first_neigh = index*NUM_NEIGH;   //访问一级邻域，SENTINEL为截至标志
	int time = 0;
	for (int k = first_neigh; neigh[k].end != SENTINEL && time<NUM_NEIGH; k++, time++) //部分点邻域大于MAX_NEIGH(20)
	{
		float ks = k_spring;
		float kd = -0.5;

		int index_neigh = neigh[k].end;
		volatile auto pos_neighData = g_pos_in[index_neigh];
		volatile auto pos_lastData = g_pos_old_in[index_neigh];
		glm::vec3 p2 = glm::vec3(pos_neighData.x, pos_neighData.y, pos_neighData.z);
		glm::vec3 p2_last = glm::vec3(pos_lastData.x, pos_lastData.y, pos_lastData.z);

		glm::vec3 v2 = (p2 - p2_last) / dt;
		glm::vec3 deltaP = pos - p2;
		if (glm::length(deltaP) == 0)
		{
			//force += glm::vec3(0.0f); continue; 
			deltaP += glm::vec3(1.0e-6);	//avoid '0'
		}  

		glm::vec3 deltaV = vel - v2;
		float dist = glm::length(deltaP); //avoid '0'


		float original_length = neigh[k].original;//glm::distance(const_pos[neigh[k].end],const_pos[index]);
		float leftTerm = -ks * (dist - original_length);
		float  rightTerm =  kd * (glm::dot(deltaV, deltaP) / dist);
		glm::vec3 springForce = (leftTerm + rightTerm)*glm::normalize(deltaP);
		
		force += springForce;
	}
	return force;

}


__global__ void compute_face_normal(glm::vec3* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int max_thread = cloth_index_size / 3;
	if (index >= max_thread)
		return;

	unsigned int f_index[3];
	for (int i = 0; i < 3; i++)
		f_index[i] = index * 3 + i;

	glm::vec3 vertex[3];
	for (int i = 0; i < 3; i++)
		vertex[i] = g_pos_in[cloth_index[f_index[i]]];  //find the fucking bug!

	glm::vec3 pos[3];
	for (int i = 0; i < 3; i++)
		pos[i] = glm::vec3(vertex[i].x, vertex[i].y, vertex[i].z);

	glm::vec3 side1, side2, normal;
	side1 = pos[1] - pos[0];
	side2 = pos[2] - pos[0];
	normal = glm::normalize(glm::cross(side1, side2));

	cloth_face[index] = normal;
}


__global__ void update_vbo_pos(glm::vec4* pos_vbo, glm::vec3* pos_cur, const unsigned int NUM_VERTICES)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	auto pos = pos_cur[index];
	pos_vbo[index] = glm::vec4(pos.x, pos.y, pos.z,1.0);
}

__global__ void verlet(glm::vec3 * g_pos_in, glm::vec3 * g_pos_old_in, glm::vec3 * g_pos_out, glm::vec3 * g_pos_old_out, glm::vec3* const_pos,
						s_spring* neigh1, s_spring* neigh2,
					    const unsigned int NUM_VERTICES,
						D_BVH bvh, glm::vec3* collision_force)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;
	
	volatile glm::vec3 posData = g_pos_in[index];
	volatile glm::vec3 posOldData = g_pos_old_in[index];


	glm::vec3 pos = glm::vec3(posData.x, posData.y, posData.z);
	glm::vec3 pos_old = glm::vec3(posOldData.x, posOldData.y, posOldData.z);
	glm::vec3 vel = (pos - pos_old) / dt;
	
	glm::vec3 gravity(gravit_x, gravit_y, gravit_z);
	glm::vec3 force = gravity*mass + vel*damp;
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh1, NUM_PER_VERTEX_SPRING_STRUCT, pos, vel,spring_structure); //计算一级邻域弹簧力
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh2, NUM_PER_VERTEX_SPRING_BEND, pos, vel,spring_bend); //计算二级邻域弹簧力

	glm::vec3 inelastic_force = glm::dot(collision_force[index], force) * collision_force[index];       //collision response force, if intersected, keep tangential
	force -= inelastic_force;
	glm::vec3 acc = force / mass;
	glm::vec3 tmp = pos;          
	pos = pos + pos - pos_old + acc * dt * dt;   
	pos_old = tmp;
	collision_response_projection(bvh, force, pos, pos_old, index, collision_force);

	g_pos_out[index] = pos;
	g_pos_old_out[index] = pos_old;
}

__global__ void compute_vbo_normal(glm::vec3* normals, unsigned int* vertex_adjface, glm::vec3* face_normal, const unsigned int NUM_VERTICES)
{

	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	//compute point normal
	glm::vec3 normal(0.0);
	int first_face_index = index * NUM_PER_VERTEX_ADJ_FACES;
	for (int i = first_face_index, time = 0; vertex_adjface[i] != SENTINEL && time < NUM_PER_VERTEX_ADJ_FACES; i++, time++)
	{
		int findex = vertex_adjface[i];
		glm::vec3 fnormal = face_normal[findex];
		normal += fnormal;
	}
	normal = glm::normalize(normal);

	normals[index] = normal;
}