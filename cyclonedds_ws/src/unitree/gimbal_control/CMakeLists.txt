cmake_minimum_required(VERSION 3.5)
project(gimbal_control_node)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(unitree_go REQUIRED)


# Include directories
include_directories(include)

# Add executable
add_executable(gimbal_control_node src/gimbal_control_node.cpp)
ament_target_dependencies(gimbal_control_node rclcpp sensor_msgs cv_bridge OpenCV unitree_go )

# Link OpenCV libraries
target_link_libraries(gimbal_control_node ${OpenCV_LIBRARIES})

# Install the executable to the correct directory
install(TARGETS gimbal_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
