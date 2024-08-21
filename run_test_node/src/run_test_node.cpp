
#include "rclcpp/rclcpp.hpp"
#include "run_test_node.hpp"

using std::placeholders::_1;

  RunTestNode::RunTestNode() : Node("RunTestNode"),pid_controller(0.1, 0.01,0.001,0.05)
  {
    state_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      sub_topic_name_, 10, std::bind(&RunTestNode::callback, this, _1));
    control_puber_ = this->create_publisher<unitree_go::msg::DogControlCommand>(pub_topic_name_, 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RunTestNode::run_step, this));
    data_ = std::make_shared<unitree_go::msg::DogReportCommon>();

  }
  void RunTestNode::run_step() {
    // std::cout<<"main";
    if(!is_updated_){
      return;
    }


      unitree_go::msg::DogControlCommand command;
      uint8_t modes[9] = {1,0,0,0,0,0,0,0,0};
      for (int i = 0; i < 9; ++i) {
          command.control_mode[i] = modes[i];
      }
      double init_heading = convert_orientation_to_eular(data_->pose.orientation);
        if (!receive_flag_) {
      receive_flag_ = true;
      target_heading_ = init_heading + M_PI / 2.0 / 10;
      }
      double yaw0 = init_heading;
      double px0 = data_->pose.position.x;
      double py0 = data_->pose.position.y;
      
      double yaw_error = target_heading_ - init_heading;
      double unit_t = 0.2;
      double time_temp = 0.0;
      // for (int i = 0; i < 30; ++i) {
      //   unitree_go::msg::PathPoint path_point;
      //   path_point.t_from_start = i * unit_t;
      //   path_point.x = data_->pose.position.x;
      //   path_point.y = data_->pose.position.y;
      //   path_point.yaw = i * yaw_error / 30  + init_heading; // 是否速度和姿态误差都需要
      //   path_point.vx = 0.0;
      //   path_point.vy = 0.0;
      //   double control_v_yaw = pid_controller.compute(i * yaw_error / 30);
      //   std::cout<<"yaw: " << path_point.yaw << " control_v_yaw: " << control_v_yaw;
      //   path_point.vyaw = control_v_yaw;
      //   command.path_point[i] = path_point;
      // }
      std::vector<unitree_go::msg::PathPoint> path;
      for (int i = 0; i < 3; i++)
        {
          unitree_go::msg::PathPoint path_point_tmp;
          time_temp += unit_t;
          // Convert trajectory commands to the initial coordinate system
          path_point_tmp.t_from_start = i * unit_t;
          path_point_tmp.x = data_->pose.position.x;
          path_point_tmp.y = data_->pose.position.y;
          path_point_tmp.yaw = i * yaw_error / 3  + init_heading;
          path_point_tmp.vx = 0.0;
          path_point_tmp.vy = 0.0;
          double control_v_yaw = pid_controller.compute(i * yaw_error / 30);
          path_point_tmp.vyaw = control_v_yaw;
          path.push_back(path_point_tmp);
          command.push_back(path_point_tmp);
          }
          
      // for (int i = 0; i < 30; i++)
      //   {
      //     unitree_go::msg::PathPoint path_point_tmp;
      //     time_temp += unit_t;
      //     // Tacking a sin path in x direction
      //     // The path is respect to the initial coordinate system
      //     float px_local = 0.5 * sin(0.5 * time_temp);
      //     float py_local = 0;
      //     float yaw_local = 0.;
      //     float vx_local = 0.5 * cos(0.5 * time_temp);
      //     float vy_local = 0;
      //     float vyaw_local = 0.;

      //     // Convert trajectory commands to the initial coordinate system
      //     path_point_tmp.t_from_start = i * unit_t;
      //     path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
      //     path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
      //     path_point_tmp.yaw = yaw_local + yaw0;
      //     path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
      //     path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
      //     path_point_tmp.vyaw = vyaw_local;
      //     command.path_point[i] = path_point_tmp;
      //     }
      // pub
      control_puber_->publish(command);
    }
  // }
//   void RunTestNode::callback(unitree_go::msg::DogReportCommon::SharedPtr data)
//   {
//     if (!success_flag_) {
//     if (!receive_flag_) {
//       receive_flag_ = true;
//       double init_heading = convert_orientation_to_eular(data->pose.orientation);
//       target_heading_ = init_heading - M_PI / 2.0;
//       init_position_x_ = data->pose.position.x;
//       init_position_y_ = data->pose.position.y;
//     }
//     // comput control value
//     double heading = convert_orientation_to_eular(data->pose.orientation);
//     double yaw_error = target_heading_ - heading;
//     std::cout << "*************************" << std::endl;
//     std::cout << "init_x: " << init_position_x_ << " " << "init_y: " << init_position_y_ << std::endl;
//     std::cout << "x: " << data->pose.position.x << " " << "y: " << data->pose.position.y << " " << "heading: " << heading << std::endl;
//     double position_error = std::sqrt((data->pose.position.x - init_position_x_) * (data->pose.position.x - init_position_x_) + \
//                             (data->pose.position.y - init_position_y_) * (data->pose.position.y - init_position_y_));
//     std::cout << "position_error: " << position_error << std::endl;
//     std::cout << "yaw_error: " << yaw_error << std::endl;
//     double control_value = pid_controller.compute(yaw_error);
//     if (abs(yaw_error) < 0.02) {
//       std::cout << "success!!!" << std::endl;
//       control_value = 0.0;
//       success_flag_ = true;
//     }

//     std::cout << "control_value: " << control_value << std::endl;
//     // assign control value
//     unitree_go::msg::DogControlCommand command;
//     uint8_t modes[9] = {0,0,0,0,1,0,0,0,1};
//     for (int i = 0; i < 9; ++i) {
//         command.control_mode[i] = modes[i];
//     }
//     geometry_msgs::msg::Twist control_point;
//     control_point.linear.x = 0.0;
//     control_point.linear.y = 0.0;
//     control_point.linear.z = control_value; // 是否需要move接口中的vyaw
//     control_point.angular.x = 0.0;
//     control_point.angular.y = 0.0;
//     control_point.angular.z = 0.0;
//     command.point = control_point;
//     // pub
//     control_puber_->publish(command);
//     }
// };
  void RunTestNode::callback(unitree_go::msg::DogReportCommon::SharedPtr data)
  {
    std::cout << "receive" << std::endl;
    is_updated_ = true;
    data_ = std::make_shared<unitree_go::msg::DogReportCommon>(*data);
    // data_ = data;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::spin(std::make_shared<RunTestNode>());
    rclcpp::shutdown();
    return 0;
}
