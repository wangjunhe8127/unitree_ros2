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
  std::string pointcloud_topic_ = "/localizer/map_cloud";
  std::string path_topic_ = "/planning/path";
  // topic callback
  void run_step();
  void loc_callback(const geometry_msgs::msg::Pose::ConstSharedPtr data);
  void waypoint_callback(const geometry_msgs::msg::Point::ConstSharedPtr data);
  void nav_status_callback(
    const std_msgs::msg::Bool::ConstSharedPtr data);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr data);
  // topic suber
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr loc_suber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_suber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_status_suber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_suber_;
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_;
  std::shared_ptr<PathPlanCore> path_planner_;
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
};