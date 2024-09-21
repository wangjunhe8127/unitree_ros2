cmake_minimum_required(VERSION 3.5)
project(generate_boundary_points_node)

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
  rclcpp
  std_msgs
  rosbag2_cpp
  geometry_msgs
  nav_msgs
  pcl_ros
  Eigen3
)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(generate_boundary_points_node_exe src/generate_boundary_points_node.cpp)
ament_target_dependencies(generate_boundary_points_node_exe ${DEPENDENCY_LIST})

install(TARGETS 
        generate_boundary_points_node_exe
        DESTINATION  lib/${PROJECT_NAME})

ament_package()
