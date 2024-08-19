#include <string>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/dog_control_command.hpp"
#include "unitree_go/msg/path_point.hpp"
#include "common.h"

class RunTestNode : public rclcpp::Node{
 public:
  RunTestNode();

 private:
  void callback(unitree_go::msg::DogReportCommon::SharedPtr data);
  std::string sub_topic_name_ = "/control/dog_report_common";
  std::string pub_topic_name_ = "/control/dog_control_command";
  rclcpp::Subscription<unitree_go::msg::DogReportCommon>::SharedPtr state_suber_;
  rclcpp::Publisher<unitree_go::msg::DogControlCommand>::SharedPtr control_puber_;
  bool receive_flag_{false};
  double target_heading_{0.0};
  PIDController pid_controller(0.1, 0.01, 0.001, 0.02); // kp, ki, kd, dt
};