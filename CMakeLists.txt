cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(simulator LANGUAGES CXX CUDA)

# deps
add_subdirectory(3rd)

# CUDA_HOST_COMPILER 的值可能会出错


# 暂时禁用
list(APPEND CUDA_NVCC_FLAGS "-rdc=true")

file(GLOB_RECURSE SRC_FILES src/*.cu src/*.cpp src/*.h)

CUDA_ADD_EXECUTABLE(simulator ${SRC_FILES})
target_compile_features(simulator PUBLIC cxx_std_11)
set_target_properties(simulator PROPERTIES 
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_ARCHITECTURES 35 50 72)
target_link_libraries(simulator 3rd_libs)