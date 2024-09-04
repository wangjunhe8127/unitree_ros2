cmake_minimum_required(VERSION 3.5)
project(control_node)

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
include_directories(../comman_convert_node/include ../comman_convert_node/include/nlohmann)
include_directories(../planning_node/include)
link_directories(src)
set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  geometry_msgs
  nav_msgs
)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(control_node_exe src/control_node.cpp ../comman_convert_node/src/ros2_sport_client.cpp)
ament_target_dependencies(control_node_exe ${DEPENDENCY_LIST})

install(TARGETS 
control_node_exe
        DESTINATION  lib/${PROJECT_NAME})

ament_package()
