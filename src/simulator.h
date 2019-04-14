#pragma once
#include "Mesh.h"
#include "./bvh/bvh.h"
#include "spring.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


class Simulator
{
public:
	Simulator();
	~Simulator();
	Simulator(Mesh& cloth,Mesh& body);
	void simulate(Mesh* cloth);
	void cuda_update_vbo(Mesh* sim_cloth);


private:
	void init_cloth(Mesh& cloth);
	void init_spring(Mesh& cloth);
	void build_bvh(Mesh& body);
	void cuda_verlet(const unsigned int numParticles);

	void get_vertex_adjface(Mesh& sim_cloth, vector<unsigned int>& CSR_R, vector<unsigned int>& CSR_C_adjface);
	void computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads);
	void swap_buffer();
	void save(string file_name);

private:
	cudaGraphicsResource* d_vbo_array_resource;  //map OPENGL array buffer to cuda
	cudaGraphicsResource* d_vbo_index_resource;  //map OPENGL index buffer to cuda

	// springs
	// i-th start index = CSR_R[i], i-th end index = CSR_R[i+1] 
	unsigned int* CSR_R_structure, *CSR_R_bend;   // adjacent structure springs for each vertex 
	s_spring* CSR_C_structure, *CSR_C_bend;        // adjacent bend springs for each vertex 

	// pre-malloc for vertices computation
	int readID, writeID;
	glm::vec3* x_cur[2];
	glm::vec3* x_last[2];

	// point to x_cur or x_last, name a straightforward name to make code more clear
	glm::vec3 * x_cur_in, *x_cur_out;
	glm::vec3 *x_last_in, *x_last_out;

	// pre-malloc for nomal computation
	glm::vec3* d_collision_force;           // store the normal of the face if collided, or set 0.0 if no collision
	unsigned int* d_CSR_R, *d_CSR_C_adjface_to_vertex;    //To compute each point's normal, we need the adjacent face indices of each point

	// used in update_vbo, just to avoid calling malloc once cuda_update_vbo called
	glm::vec3* d_face_normals;        // face(triangle) normal	
public:

	//for bvh tree
	BVHAccel* cuda_bvh;
};

