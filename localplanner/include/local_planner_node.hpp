#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common.hpp"
#include "path_planner.hpp"

class LocalPlannerNode : public rclcpp::Node{
 public:
  LocalPlannerNode();

 private:
  // ros ms
  int ros_ms{20};
  // topic name
  std::string loc_topic_ = "/routing/loc";
  std::string waypoint_topic_ = "/routing/waypoint";
  std::string nav_status_topic_ = "/routing/nav_status";
  std::string pointcloud_topic_ = "map_cloud";
  std::string path_topic_ = "/planning/path";
  // topic callback
  void run_step();
  void loc_callback();
  void waypoint_callback();
  void nav_status_callback();
  void pointcloud_callback();
  // topic suber
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr loc_suber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_suber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_status_suber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_suber_;
  // topic puber
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_puber_;
  // timer
  rclcpp::TimerBase::SharedPtr run_timer_;
  // var
  double joySpeed{1.0};
  double odomTime{0.0};
  double adjacentRange{3.0};

  bool receive_loc_{false};
  bool receive_goal_{false};
  bool receive_nav_status_{false};
  bool receive_pointcloud_{false};

  bool nav_status_{true};
  unitree::planning::StatePoint loc_point_;
  unitree::planning::StatePoint goal_point_;
  nav_msgs::msg::Path nav_path_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_(new pcl::PointCloud<pcl::PointXYZI>());
  std::shared_ptr<PathPlanCore> path_planner_;
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
};