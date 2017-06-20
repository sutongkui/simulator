#pragma once

/*check error code of cudaMalloc and print out if needed*/
#define safe_cuda(CODE)\
 {\
  cudaError_t err = CODE;\
  if(err != cudaSuccess) {\
    std::cout<<"CUDA error:"<<cudaGetErrorString(err)<<std::endl;\
 }\
}


/**
* alloc a memory on gpu and copy data from cpu to gpu.
*/
inline void copyFromCPUtoGPU(void** dst, void* src, int size)
{
	cudaMalloc(dst, size);
	safe_cuda(cudaMemcpy(*dst, src, size, cudaMemcpyHostToDevice));
}

/**
* alloc a memory on cpu and copy data from gpu to cpu.
*/
inline void copyFromGPUtoCPU(void** dst, void* src, int size)
{
	*dst = malloc(size);
	safe_cuda(cudaMemcpy(*dst, src, size, cudaMemcpyDeviceToHost));
}