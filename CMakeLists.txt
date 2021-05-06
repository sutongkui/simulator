cmake_minimum_required(VERSION 2.8)

project(simulator)

set(CUDA_PROPAGATE_HOST_FLAGS ON)
set(CUDA_SEPARABLE_COMPILATION ON)
find_package(CUDA REQUIRED)
list(APPEND CUDA_NVCC_FLAGS "-rdc=true")
list(APPEND CUDA_NVCC_FLAGS_DEBUG "-G")

include_directories("${CMAKE_SOURCE_DIR}/Include")
link_directories("${CMAKE_SOURCE_DIR}/lib" )

file(GLOB PATHS_SRC src/*.cu src/*.cpp)
file(GLOB PATHS_SRC_BVH src/bvh/*.cu src/bvh/*.cpp)

CUDA_ADD_EXECUTABLE(test ${PATHS_SRC} ${PATHS_SRC_BVH})
set_property(TARGET test PROPERTY CXX_STANDARD 11)
target_link_libraries(test FreeImage)
target_link_libraries(test ${CUDA_cudadevrt_LIBRARY})
