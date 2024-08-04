cmake_minimum_required(VERSION 3.5)
project(state_convert_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)
link_directories(src)
set (
  DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
  rosbag2_cpp
)

find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(state_convert_node_exe src/state_convert_node.cpp)
ament_target_dependencies(state_convert_node_exe ${DEPENDENCY_LIST})

install(TARGETS 
        state_convert_node_exe
        DESTINATION)

ament_package()
