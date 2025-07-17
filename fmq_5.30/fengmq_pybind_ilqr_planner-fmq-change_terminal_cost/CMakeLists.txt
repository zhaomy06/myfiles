cmake_minimum_required(VERSION 3.16)
project(example)

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 添加这行来启用-fPIC
# 查找必需的包（Protobuf 和 Eigen）
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR}/build/proto)
include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(proto)
add_subdirectory(ilqr_lib)

set(PYTHON_INCLUDE_DIRS "/usr/include/python3.8/Python.h")
set(PYTHON_LIBRARIES "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so") # 或者对于Windows, libpython3.x.lib

file(GLOB PLANNER_SOURCE_FILE "planning_core/src/*.cpp")
# 找到Python库和pybind11
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
find_package(pybind11 REQUIRED)

add_executable(test ${PLANNER_SOURCE_FILE})
target_link_libraries (test PUBLIC Eigen3::Eigen ilqr_lib proto)

add_library(ilqr_planner ${PLANNER_SOURCE_FILE})
target_link_libraries (ilqr_planner PUBLIC Eigen3::Eigen ilqr_lib proto)
target_compile_options(ilqr_planner PRIVATE -fPIC)

pybind11_add_module(pybind_test ./pybind_cpp/pybind_test.cpp)
target_link_libraries (pybind_test PRIVATE Eigen3::Eigen ilqr_planner)


