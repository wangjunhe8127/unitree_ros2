#include <string>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class StateConvertNode : public rclcpp::Node{
 public:
  StateConvertNode();

 private:
  void high_callback(unitree_go::msg::SportModeState::SharedPtr data);
  std::string high_topic_name_ = "lf/sportmodestate";
  std::string pub_topic_name_ = "/control/dog_report_common";
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr high_suber_;
  rclcpp::Publisher<unitree_go::msg::DogReportCommon>::SharedPtr state_puber_;
};