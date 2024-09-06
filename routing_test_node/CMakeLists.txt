cmake_minimum_required(VERSION 3.5)
project(routing_test_node)

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
include_directories(../planning_node/include)
link_directories(src)
set (
  DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
  rosbag2_cpp
  geometry_msgs
  tf2_ros
  nav_msgs
)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(routing_test_node_exe src/routing_test_node.cpp)
ament_target_dependencies(routing_test_node_exe ${DEPENDENCY_LIST})

install(TARGETS 
        routing_test_node_exe
        DESTINATION  lib/${PROJECT_NAME})

ament_package()
