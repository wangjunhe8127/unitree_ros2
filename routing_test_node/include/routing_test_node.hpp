#pragma once
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/routing.hpp"
#include "common.hpp"
class RoutingTesetNode : public rclcpp::Node{
 public:
  RoutingTesetNode();
 private:
  void run_step();
  std::string loc_topic_name_ = "/control/dog_report_common";
  std::string routing_topic_name_ = "/control/routing";
  void loc_callback(unitree_go::msg::DogReportCommon::SharedPtr data);
  rclcpp::Subscription<unitree_go::msg::DogReportCommon>::SharedPtr loc_suber_;
  rclcpp::Publisher<unitree_go::msg::Routing>::SharedPtr routing_puber_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  bool receive_loc_{false};
  std::vector<std::pair<double, double>> ref_;
  std::vector<std::pair<double, double>> left_;
  std::vector<std::pair<double, double>> right_;
  double init_heading_{0.0};
  double init_x_{0.0};
  double init_y_{0.0};
  double dis_finish_th_{0.15};
  double yaw_finish_th_{0.06};
  geometry_msgs::msg::Point end_point_;
};
