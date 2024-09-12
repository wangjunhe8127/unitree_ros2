
#include "command_convert_node.hpp"
using std::placeholders::_1;
  CommandConvertNode::CommandConvertNode() : Node("CommandConvertNode")
  {

    convert_list_.push_back(convert_trajectory_);
    convert_list_.push_back(convert_speed_level_);
    convert_list_.push_back(convert_body_height_);
    convert_list_.push_back(convert_damp_);
    convert_list_.push_back(convert_balance_stand_);
    convert_list_.push_back(convert_stop_move_);
    convert_list_.push_back(convert_stand_up_);
    convert_list_.push_back(convert_recovery_stand_);
    convert_list_.push_back(convert_position_);
    convert_list_.push_back(convert_euler_);
    control_suber_ = this->create_subscription<unitree_go::msg::DogControlCommand>(
      sub_topic_name_, 10, std::bind(&CommandConvertNode::control_callback, this, _1));
    control_puber_ = this->create_publisher<unitree_api::msg::Request>(pub_topic_name_, 10);
  }
  void CommandConvertNode::control_callback(unitree_go::msg::DogControlCommand::SharedPtr data)
  {
    for (int i = 0; i < 9; i++) {
      if (data->control_mode[i] == 1) {
        convert_list_[i]->Process(data, req_);
        control_puber_->publish(req_);
        if (i == 8) {
          convert_list_[9]->Process(data, req_);
          control_puber_->publish(req_);
        }
      }
    }
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals
    rclcpp::spin(std::make_shared<CommandConvertNode>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}
