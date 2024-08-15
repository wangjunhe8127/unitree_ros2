cmake_minimum_required(VERSION 3.5)
project(gs_transfer_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include and link directories
include_directories(include)
include_directories(proto)
link_directories(src)

# Dependency list (excluding protobuf)
set(DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
  rosbag2_cpp
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Protobuf REQUIRED)  # Finding Protobuf as an external library

# Add executable
add_executable(gs_transfer_node src/gs_transfer_node.cpp src/state_data_mapper.cpp proto/state_datas.pb.cc)
ament_target_dependencies(gs_transfer_node ${DEPENDENCY_LIST})

# Link Protobuf to your executable
target_link_libraries(gs_transfer_node ${Protobuf_LIBRARIES})

# generate proto
file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
protobuf_generate(LANGUAGE cpp TARGET gs_transfer_node PROTOS ${PROTO_FILES} PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Include the generated files' directory
target_include_directories(gs_transfer_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Install the executable to the correct directory
install(TARGETS gs_transfer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
