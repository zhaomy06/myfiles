cmake_minimum_required(VERSION 3.16)
project(speed_planner)

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 添加这行来启用-fPIC
# 查找必需的包（Protobuf 和 Eigen）
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR}/build/proto)
include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(proto)

set(PYTHON_INCLUDE_DIRS "/usr/include/python3.8/Python.h")
set(PYTHON_LIBRARIES "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so") # 或者对于Windows, libpython3.x.lib

file(GLOB PLANNER_SOURCE_FILE "speed_planner/*.cc" "speed_planner/common/*.cc" "test.cc")
# 找到Python库和pybind11
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
find_package(pybind11 REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_path(IPOPT_INCLUDE_DIRS NAMES "coin/IpTNLP.hpp")

if(NOT IPOPT_INCLUDE_DIRS)
  message(FATAL_ERROR "Ipopt includes were not found!")
endif()

find_library(IPOPT_LIBRARIES NAMES "ipopt")

if(NOT IPOPT_LIBRARIES)
  message(FATAL_ERROR "Ipopt library was not found!")
endif()

message("IPOPT Include dirs: ${IPOPT_INCLUDE_DIRS}")
message("IPOPT libraries: ${IPOPT_LIBRARIES}")

include_directories(${IPOPT_INCLUDE_DIRS})

add_executable(ipopt_test ${PLANNER_SOURCE_FILE})

# Link against the Ipopt library.
target_link_libraries(ipopt_test ${IPOPT_LIBRARIES} proto osqp::osqp Eigen3::Eigen)


add_library(speed_planner_lib ${PLANNER_SOURCE_FILE})
target_link_libraries (speed_planner_lib PUBLIC Eigen3::Eigen proto osqp::osqp ${IPOPT_LIBRARIES})
target_compile_options(speed_planner_lib PRIVATE -fPIC)

pybind11_add_module(speed_planner_pybind speed_planner_pybind/speed_planner_pybind.cc)
target_link_libraries (speed_planner_pybind PRIVATE speed_planner_lib)



