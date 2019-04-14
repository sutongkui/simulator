
#include "simulator.h"
#include "spring.h"
#include "Mesh.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <iostream>
#include <fstream>

#include "sim_parameter.h"
#include "watch.h"
#include "common.h"

using namespace std;

__global__ void compute_face_normal(glm::vec3* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face);   //update cloth face normal
__global__ void verlet(glm::vec3 * g_pos_in, glm::vec3 * g_pos_old_in, glm::vec3 * g_pos_out, glm::vec3 * g_pos_old_out,
						s_spring* neigh1, s_spring* neigh2,
					  const unsigned int NUM_VERTICES,
					  D_BVH bvh, glm::vec3* d_collision_force);  //verlet intergration
__global__ void update_vbo_pos(glm::vec4* pos_vbo, glm::vec3* pos_cur, const unsigned int NUM_VERTICES);
__global__ void compute_vbo_normal(glm::vec3* normals, unsigned int* vertex_adjface, glm::vec3* face_normal, const unsigned int NUM_VERTICES);

Simulator::Simulator()
{
	
}

Simulator::~Simulator()
{
	cudaFree(x_cur[0]);
	cudaFree(x_cur[1]);
	cudaFree(x_last[0]);
	cudaFree(x_last[1]);
	cudaFree(d_collision_force);
	cudaFree(d_adjface_to_vertex);
	cudaFree(d_face_normals);

	cudaFree(d_adj_structure_spring);
	cudaFree(d_adj_bend_spring);

	delete cuda_bvh;
}

Simulator::Simulator(Mesh& sim_cloth, Mesh& body) :readID(0), writeID(1)
{
	init_cloth(sim_cloth);
	init_spring(sim_cloth);
	build_bvh(body);
}

void Simulator::init_cloth(Mesh& sim_cloth)
{
	// \d_vbo_array_resource points to cloth's array buffer  
	safe_cuda(cudaGraphicsGLRegisterBuffer(&d_vbo_array_resource, sim_cloth.vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard));   	//register vbo


	//set heap size, the default is 8M
	size_t heap_size = 256 * 1024 * 1024;  
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	// Send the cloth's vertices to GPU
	const unsigned int vertices_bytes = sizeof(glm::vec3) * sim_cloth.vertices.size();
	safe_cuda(cudaMalloc((void**)&x_cur[0], vertices_bytes));			 // cloth vertices
	safe_cuda(cudaMalloc((void**)&x_cur[1], vertices_bytes));			 // cloth vertices
	safe_cuda(cudaMalloc((void**)&x_last[0], vertices_bytes));	 // cloth old vertices
	safe_cuda(cudaMalloc((void**)&x_last[1], vertices_bytes));	 // cloth old vertices
	safe_cuda(cudaMalloc((void**)&d_collision_force, sizeof(glm::vec3) * sim_cloth.vertices.size()));  //collision response force
	safe_cuda(cudaMemset(d_collision_force, 0, sizeof(glm::vec3) * sim_cloth.vertices.size()));    //initilize to 0

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];

	vector<glm::vec3> tem_vertices(sim_cloth.vertices.size());
	for (int i=0;i< sim_cloth.vertices.size();i++)
	{
		tem_vertices[i] = glm::vec3(sim_cloth.vertices[i]);   // glm::vec4 -> glm::vec3
	}

	safe_cuda(cudaMemcpy(x_cur[0], &tem_vertices[0], vertices_bytes, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(x_last[0], &tem_vertices[0], vertices_bytes, cudaMemcpyHostToDevice));

	//计算normal所需的数据：每个点邻接的面的索引 + 每个面的3个点的索引
	vector<unsigned int> vertex_adjface;
	get_vertex_adjface(sim_cloth, vertex_adjface);
	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //每个点邻接的面的索引
	safe_cuda(cudaMalloc((void**)&d_adjface_to_vertex, vertex_adjface_bytes));
	safe_cuda(cudaMemcpy(d_adjface_to_vertex, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice));
	safe_cuda(cudaMalloc((void**)&d_face_normals, sizeof(glm::vec3) * sim_cloth.faces.size()));    //face normal
	

	safe_cuda(cudaGraphicsGLRegisterBuffer(&d_vbo_index_resource, sim_cloth.vbo.index_buffer, cudaGraphicsMapFlagsWriteDiscard));   	//register vbo
}

void Simulator::init_spring(Mesh& sim_cloth)
{
	// Construct structure and bend springs in GPU
	Springs springs(&sim_cloth);
	vector<s_spring> cpu_str_spring;
	vector<s_spring> cpu_bend_spring;


	springs.serialize_structure_spring(cpu_str_spring);
	springs.serialize_bend_spring(cpu_bend_spring);

	safe_cuda(cudaMalloc((void**)&d_adj_structure_spring, cpu_str_spring.size() * sizeof(s_spring)));
	safe_cuda(cudaMalloc((void**)&d_adj_bend_spring, cpu_bend_spring.size() * sizeof(s_spring)));
	safe_cuda(cudaMemcpy(d_adj_structure_spring, &cpu_str_spring[0], cpu_str_spring.size() * sizeof(s_spring), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(d_adj_bend_spring, &cpu_bend_spring[0], cpu_bend_spring.size() * sizeof(s_spring), cudaMemcpyHostToDevice));

}

void Simulator::build_bvh(Mesh& body)
{
	stop_watch watch;
	watch.start();
	Mesh bvh_body = body;   // for bvh consttruction
	bvh_body.vertex_extend(0.003);

	watch.start();
	cuda_bvh = new BVHAccel(bvh_body);
	watch.stop();
	cout << "bvh build done free time elapsed: " << watch.elapsed() << "us" << endl;
}


void Simulator::simulate(Mesh* sim_cloth)
{
	//cuda kernel compute .........
	
	cuda_verlet(sim_cloth->vertices.size());

	cuda_update_vbo(sim_cloth);     // update array buffer for opengl

	swap_buffer();
}

void Simulator::get_vertex_adjface(Mesh& sim_cloth, vector<unsigned int>& vertex_adjface)
{
	vector<vector<unsigned int>> adjaceny(sim_cloth.vertices.size());
	for(int i=0;i<sim_cloth.faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth.faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	// 每个点最大包含20个邻近面，不足者以SENTINEL作为结束标志
	vertex_adjface.resize(sim_cloth.vertices.size()*NUM_PER_VERTEX_ADJ_FACES);
	for(int i=0;i<adjaceny.size();i++)
	{
		int j;
		for(j=0;j<adjaceny[i].size() && j<NUM_PER_VERTEX_ADJ_FACES;j++)
		{
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = adjaceny[i][j];
		}
		if(NUM_PER_VERTEX_ADJ_FACES>adjaceny[i].size())
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = SENTINEL;                  //Sentinel
	}
}

void Simulator::cuda_verlet(const unsigned int numParticles)
{
	unsigned int numThreads, numBlocks;
	
	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(x_cur_in,x_last_in, x_cur_out, x_last_out,
										d_adj_structure_spring,d_adj_bend_spring,							
										numParticles,
										*cuda_bvh->d_bvh, d_collision_force);

	// stop the CPU until the kernel has been executed
	safe_cuda(cudaDeviceSynchronize());
}

void Simulator::cuda_update_vbo(Mesh* sim_cloth)
{
	unsigned int numParticles = sim_cloth->vertices.size();

	size_t num_bytes;
	glm::vec4* d_vbo_vertex;           //point to vertex address in the OPENGL buffer
	glm::vec3* d_vbo_normal;           //point to normal address in the OPENGL buffer
	unsigned int* d_adjvertex_to_face;    // the order like this: f0(v0,v1,v2) -> f1(v0,v1,v2) -> ... ->fn(v0,v1,v2)
	
	safe_cuda(cudaGraphicsMapResources(1, &d_vbo_array_resource));
	safe_cuda(cudaGraphicsMapResources(1, &d_vbo_index_resource));
	safe_cuda(cudaGraphicsResourceGetMappedPointer((void **)&d_vbo_vertex, &num_bytes, d_vbo_array_resource));
	safe_cuda(cudaGraphicsResourceGetMappedPointer((void **)&d_adjvertex_to_face, &num_bytes, d_vbo_index_resource));

	d_vbo_normal = (glm::vec3*)((float*)d_vbo_vertex + 4 * sim_cloth->vertices.size() + 2 * sim_cloth->tex.size());   // 获取normal位置指针	

	unsigned int numThreads, numBlocks;

	// update vertex position
	computeGridSize(numParticles, 512, numBlocks, numThreads);
	update_vbo_pos << < numBlocks, numThreads >> > (d_vbo_vertex, x_cur_out, numParticles);
	safe_cuda(cudaDeviceSynchronize());  	// stop the CPU until the kernel has been executed

	// we need to compute face normal before computing vbo normal
	computeGridSize(sim_cloth->faces.size(), 512, numBlocks, numThreads);
	compute_face_normal << <numBlocks, numThreads >> > (x_cur_in, d_adjvertex_to_face, sim_cloth->vertex_indices.size(), d_face_normals);
	safe_cuda(cudaDeviceSynchronize());

	// update vertex normal
	computeGridSize(numParticles, 1024, numBlocks, numThreads);
	compute_vbo_normal << < numBlocks, numThreads >> > (d_vbo_normal, d_adjface_to_vertex, d_face_normals ,numParticles);
	safe_cuda(cudaDeviceSynchronize());

	safe_cuda(cudaGraphicsUnmapResources(1, &d_vbo_index_resource));
	safe_cuda(cudaGraphicsUnmapResources(1, &d_vbo_array_resource));
}

void Simulator::save(string file_name)
{
}

void Simulator::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void Simulator::swap_buffer()
{
	swap(readID, writeID);

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];
}


