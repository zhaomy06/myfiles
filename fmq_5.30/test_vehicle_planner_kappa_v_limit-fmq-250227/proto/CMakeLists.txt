# proto_lib/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

project(proto_lib)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
message(STATUS "Protobuf_INCLUDE_DIRS: ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Protobuf_LIBRARIES: ${Protobuf_LIBRARIES}")

# 获取所有 .proto 文件
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

# 设置 Proto 文件的输出路径和禁止追加原始路径
set(PROTOBUF_GENERATE_CPP_APPEND_PATH FALSE)

# 使用 protoc 生成 .proto 文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# 创建一个库（静态或共享）
add_library(proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(proto PUBLIC ${Protobuf_LIBRARIES})
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

protobuf_generate_python(
  PROTO_PYS
  ${PROTO_FILES}
)
add_custom_target(${PROJECT_NAME}_py ALL DEPENDS ${PROTO_PYS})