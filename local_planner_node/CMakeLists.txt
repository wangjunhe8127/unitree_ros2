cmake_minimum_required(VERSION 3.5)
project(local_planner_node)

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
  nav_msgs
  geometry_msgs
  sensor_msgs
  unitree_go
  tf2_ros
  tf2_geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  unitree_go
  tf2
  rclcpp
  std_msgs
)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(local_planner_node_exe src/local_planner_node.cpp)
ament_target_dependencies(local_planner_node_exe ${DEPENDENCY_LIST})

install(TARGETS 
        local_planner_node_exe
        DESTINATION  lib/${PROJECT_NAME})

ament_package()
