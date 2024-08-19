
#include "rclcpp/rclcpp.hpp"
#include "run_test_node.hpp"

using std::placeholders::_1;

  RunTestNode::RunTestNode() : Node("RunTestNode")
  {
    state_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      sub_topic_name_, 10, std::bind(&RunTestNode::callback, this, _1));
    control_puber_ = this->create_publisher<unitree_go::msg::DogControlCommand>(pub_topic_name_, 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&RunTestNode::main, this));
  }

  void RunTestNode::callback(unitree_go::msg::DogReportCommon::SharedPtr data)
  {
    if (!receive_flag_) {
      receive_flag_ = true;
      double init_heading = convert_orientation_to_eular(data->pose.orientation);
      target_heading_ = init_heading + M_PI / 2.0;
    }
    // comput control value
    double heading = convert_orientation_to_eular(data->pose.orientation);
    double yaw_error = target_heading_ - heading;
    if (heading_error < 0.2) {
      std::cout << "yaw_error: " << yaw_error << std::endl;
    }
    control_value = pid_controller.compute(yaw_error);
    // assign control value
    unitree_go::msg::DogControlCommand command;
    uint8_t modes[9] = {0,0,0,0,1,0,0,0,1};
    for (int i = 0; i < 9; ++i) {
        command.control_mode[i] = modes[i];
    }
    geometry_msgs::msg::Twist control_point;
    control_point.linear.x = 0.0;
    control_point.linear.y = 0.0;
    control_point.linear.z = 0.0; // 是否需要move接口中的vyaw
    control_point.angular.x = 0.0;
    control_point.angular.y = 0.0;
    control_point.angular.z = control_value;
    command.point = control_point;
    // pub
    control_puber_->publish(command);
};
//   void RunTestNode::callback(unitree_go::msg::DogReportCommon::SharedPtr data)
//   {
//     if (!receive_flag_) {
//       receive_flag_ = true;
//       unitree_go::msg::DogControlCommand command;
//       uint8_t modes[9] = {1,0,0,0,1,0,0,0,0};
//       for (int i = 0; i < 9; ++i) {
//           command.control_mode[i] = modes[i];
//       }
//       double init_heading = convert_orientation_to_eular(data->pose.orientation);
//       target_heading_ = init_heading + M_PI / 2.0;
//       double yaw_error = target_heading_ - init_heading;
//       double unit_t = 0.1;
//       for (int i = 0; i < 30; ++i) {
//         unitree_go::msg::PathPoint path_point;
//         path_point.timeFromStart = i * unit_t;
//         path_point.x = data->pose.position.x;
//         path_point.y = data->pose.position.y;
//         path_point.yaw = i * yaw_error / 30  + init_heading; // 是否速度和姿态误差都需要
//         path_point.vx = 0.0;
//         path_point.vy = 0.0;
//         control_v_yaw = pid_controller.compute(i * yaw_error / 30);
//         path_point.vyaw = control_v_yaw;
//         command.path_point.push_back(path_point);
//       }
//       // pub
//       control_puber_->publish(command);
//     }
// };
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::spin(std::make_shared<RunTestNode>());
    rclcpp::shutdown();
    return 0;
}
