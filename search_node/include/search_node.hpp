#pragma once
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include <string>
#include <vector>
#include "common.hpp"

namespace planning {
namespace path {
class SearchNode : public rclcpp::Node {
public:
  SearchNode();

private:
  void state_callback(unitree_go::msg::DogReportCommon::SharedPtr data);
  void map_callback(unitree_go::msg::HeightMap::SharedPtr data);
  std::string map_topic_name_ = "/utlidar/height_map_array";
  std::string state_topic_name_ = "/control/dog_report_common";
  rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr map_suber_;
  rclcpp::Subscription<unitree_go::msg::DogReportCommon>::SharedPtr
      state_suber_;
  bool state_receive_{false};
  bool map_receive_{false};
  bool routing_receive_{true};
  SearchData search_data_;
  SearchCore search_core_;
  SearchOut search_result_;
  bool search_flag_{false};
};
} // namespace path
} // namespace planning