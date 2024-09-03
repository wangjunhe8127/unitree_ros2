#pragma once
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/height_map.hpp"
#include "unitree_go/msg/routing.hpp"
#include <string>
#include <vector>
#include "common.hpp"
#include "search_core.hpp"
#include "motion_core.hpp"
namespace unitree {
namespace planning {
class PlanningNode : public rclcpp::Node {
public:
  PlanningNode();

private:
  void run_step();
  bool SameEndPoint();
  bool CheckReachEnd();
  void state_callback(unitree_go::msg::DogReportCommon::SharedPtr data);
  void map_callback(unitree_go::msg::HeightMap::SharedPtr data);
  void routing_callback(unitree_go::msg::Routing::SharedPtr data);
  std::string map_topic_name_ = "/utlidar/height_map_array";
  std::string state_topic_name_ = "/control/dog_report_common";
  std::string routing_topic_name_ = "/control/routing";
  std::string planning_topic_name_ = "/control/dog_control_command";
  rclcpp::Publisher<unitree_go::msg::DogControlCommand>::SharedPtr planning_puber_;

  rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr map_suber_;
  rclcpp::Subscription<unitree_go::msg::DogReportCommon>::SharedPtr
      state_suber_;
  rclcpp::Subscription<unitree_go::msg::Routing>::SharedPtr
      routing_suber_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  std::shared_ptr<SearchCore> search_core_;
  std::shared_ptr<MotionCore> motion_core;
  bool state_receive_{false};
  bool map_receive_{false};
  bool routing_receive_{true};
  bool search_flag_{false};
  bool reach_end_flag_{false};
  SearchIn search_in_;
  PlanningResult planning_result_;
  StatePoint last_end_point_;
  double reach_end_th_{0.1}; // 需要和routing模块中的值对齐，否则会导致不能切换航点
  double target_heading_{0.0};
  int timer_time_{50};
  bool routing_finish_{false};

};
} // namespace path
} // namespace planning