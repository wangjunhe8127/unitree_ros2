#pragma once
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/height_map.hpp"
#include <string>
#include <vector>
#include "common.hpp"
#include "search_core.hpp"

namespace unitree {
namespace planning {
class PlanningNode : public rclcpp::Node {
public:
  PlanningNode();

private:
  void run_step();
  void state_callback(unitree_go::msg::DogReportCommon::SharedPtr data);
  void map_callback(unitree_go::msg::HeightMap::SharedPtr data);
  std::string map_topic_name_ = "/utlidar/height_map_array";
  std::string state_topic_name_ = "/control/dog_report_common";
  rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr map_suber_;
  rclcpp::Subscription<unitree_go::msg::DogReportCommon>::SharedPtr
      state_suber_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  std::shared_ptr<SearchCore> search_core_;
  bool state_receive_{false};
  bool map_receive_{false};
  bool routing_receive_{true};
  bool search_flag_{false};
  SearchIn search_in_;
  SearchOut search_result_;

};
} // namespace path
} // namespace planning