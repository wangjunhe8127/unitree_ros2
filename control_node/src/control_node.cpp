
#include "rclcpp/rclcpp.hpp"
#include "control_node.hpp"
ControlNode::ControlNode() : Node("ControlNode"){
  // loc_suber_ = this->create_subscription<geometry_msgs::msg::Pose>(
  //     loc_topic_, 10, std::bind(&ControlNode::loc_callback, this, std::placeholders::_1));
  path_suber_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&ControlNode::path_callback, this, std::placeholders::_1));
  stop_suber_ = this->create_subscription<std_msgs::msg::Bool>(
      stop_topic_, 10,
      std::bind(&ControlNode::stop_callback, this, std::placeholders::_1));
  control_puber_ = this->create_publisher<unitree_api::msg::Request>(control_topic_, 10);
  run_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(ros_ms),
                                std::bind(&ControlNode::run_step, this));
  controller_ = std::make_shared<Controller>();
}
void ControlNode::run_step() {
  if (receive_path_ && receive_stop_) {
    controller_->process(loc_point_, path_, stop_, req);
    control_puber_->publish(req);
  }
}
// void ControlNode::loc_callback(
//     const geometry_msgs::msg::Pose::ConstSharedPtr data) {
//   receive_loc_ = true;
//   loc_point_.x = data->position.x;
//   loc_point_.y = data->position.y;
//   loc_point_.z = data->position.z;
//   loc_point_.heading =
//       unitree::planning::convert_orientation_to_eular(data->orientation);
// }
void ControlNode::path_callback(
    const nav_msgs::msg::Path::ConstSharedPtr data) {
  receive_path_ = true;
  int pathSize = data->poses.size();
  path_.poses.clear();
  path_.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path_.poses[i].pose.position.x = data->poses[i].pose.position.x;
    path_.poses[i].pose.position.y = data->poses[i].pose.position.y;
    path_.poses[i].pose.position.z = data->poses[i].pose.position.z;
  }
}
void ControlNode::stop_callback(const std_msgs::msg::Bool::ConstSharedPtr data) {
  receive_stop_ = true;
  stop_ = (data->data == false);
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals
    rclcpp::spin(std::make_shared<ControlNode>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}
