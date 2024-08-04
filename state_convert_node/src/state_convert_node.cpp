
#include "rclcpp/rclcpp.hpp"
#include "state_convert_node.hpp"

#define HIGH_FREQ 0       // Set 1 to subscribe to motion states with high frequencies (500Hz)
using std::placeholders::_1;

  StateConvertNode::StateConvertNode() : Node("StateConvertNode")
  {
    if (HIGH_FREQ) {
        // low_topic_name_ = "lowstate";
        high_topic_name_ = "sportmodestate";
    }
    high_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        high_topic_name_, 10, std::bind(&StateConvertNode::high_callback, this, _1));
    // low_suber_ = this->create_subscription<unitree_go::msg::LowState>(
    //     low_topic_name_, 10, std::bind(&StateConvertNode::low_callback, this, _1));
    state_puber_ = this->create_publisher<unitree_go::msg::DogReportCommon>(pub_topic_name_, 10);
  }
  void StateConvertNode::high_callback(unitree_go::msg::SportModeState::SharedPtr data)
  {
    unitree_go::msg::DogReportCommon command;
    command.pose.position.x = data->position[0];
    command.pose.position.y = data->position[0];
    command.pose.position.z = data->position[0];
    command.pose.orientation.x = data->imu_state.quaternion[1];
    command.pose.orientation.y = data->imu_state.quaternion[2];
    command.pose.orientation.z = data->imu_state.quaternion[3];
    command.pose.orientation.w = data->imu_state.quaternion[0];
    command.speed.vx = data->velocity[0];
    command.speed.vy = data->velocity[1];
    command.speed.vz = data->velocity[2];
    command.speed.v_yaw = data->yaw_speed;
    command.body_height = data->body_height;
    command.gyroscope.x = data->imu_state.gyroscope[0];
    command.gyroscope.y = data->imu_state.gyroscope[1];
    command.gyroscope.z = data->imu_state.gyroscope[2];
    command.accelerometer.x = data->imu_state.accelerometer[0];
    command.accelerometer.y = data->imu_state.accelerometer[1];
    command.accelerometer.z = data->imu_state.accelerometer[2];
    state_puber_->publish(command);
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals
    rclcpp::spin(std::make_shared<StateConvertNode>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}
