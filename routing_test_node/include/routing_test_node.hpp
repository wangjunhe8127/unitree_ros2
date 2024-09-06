#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/routing.hpp"
#include "common.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "motion_core.hpp"
class RoutingTesetNode : public rclcpp::Node{
 public:
  RoutingTesetNode();
 private:
  void run_step();

  bool load_waypoints(const std::string &waypoint_path);
  bool check_waypoint_finish();
  void loc_callback();
  void send_waypoint();
  void control_r();
  std::string loc_topic_name_ = "/routing/loc";
  std::string waypoint_topic_name_ = "/routing/waypoint";
  std::string nav_status_topic_name_ = "/routing/nav_status";
  std::string r_control_topic_name_ = "/control/dog_control_command";
  // std::string boundary_topic_name_ = "/routing/boundary";
  rclcpp::TimerBase::SharedPtr run_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr loc_puber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_puber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_status_puber_;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  // msg
  std_msgs::msg::Bool nav_status_;
  nav_msgs::msg::Odometry loc_pose_;
  geometry_msgs::msg::Point waypoint_; // x,y,yaw分别填充x,y,z

  int waypoint_idx_{0};
  std::vector<std::vector<double>> waypoints_;
  double diff_dis_th_{0.1};
  double diff_yaw_th_{0.1};
  bool receive_loc_{false};
  bool arrive_end_{false};

  std::shared_ptr<unitree::planning::MotionCore> motion_core;
  unitree_go::msg::DogControlCommand command_;
  rclcpp::Publisher<unitree_go::msg::DogControlCommand>::SharedPtr r_puber_;
};
