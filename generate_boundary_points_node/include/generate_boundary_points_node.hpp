#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
class GenerateBoundaryPointsNode : public rclcpp::Node{
 public:
  GenerateBoundaryPointsNode();
 private:
  void run_step();
  bool load_waypoints(const std::string &waypoint_path);
  rclcpp::TimerBase::SharedPtr run_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_point_puber_;
  std::vector<Eigen::Vector3d> boundary_points_;
  double unit_s_{0.02};
  sensor_msgs::msg::PointCloud2 output_cloud_;
};
