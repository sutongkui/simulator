cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(simulator LANGUAGES CUDA C CXX)

# deps
add_subdirectory(3rd)

file(GLOB_RECURSE SRC_FILES src/*.cu src/*.cpp src/*.h)

add_executable(simulator ${SRC_FILES})
target_compile_features(simulator PUBLIC cxx_std_11)
set_target_properties(simulator PROPERTIES 
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_ARCHITECTURES 35 50 75
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>") 
target_link_libraries(simulator 3rd_libs)