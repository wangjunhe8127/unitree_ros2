#pragma once
#include "common.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg/dog_control_command.hpp"
#include "unitree_go/msg/path_point.hpp"
#include <vector>
namespace unitree {
namespace planning {
class MotionCore {
public:
  MotionCore(double kp, double ki, double kd, double dt)
      : kp_(kp), ki_(ki), kd_(kd), dt_(dt) {
      // unitree_go::msg::PathPoint path_point;
      //   path_point.t_from_start = 0.0;
      //   path_point.x = 0.0;
      //   path_point.y = 0.0;
      //   path_point.yaw = 0.0;
      //   path_point.vx = 0.0;
      //   path_point.vy = 0.0;
      //   path_point.vyaw = 0.0;
      //   last_command_.path_point.push_back(path_point);
      //   last_command_.path_point.push_back(path_point);
      }
  //*****************PoseControl*************//
  unitree_go::msg::DogControlCommand PoseControl(double current, double target) {
    unitree_go::msg::DogControlCommand command;
    double error = target - current;
    std::cout << "control_error:" << error << std::endl;
    std::cout << "control_current:" << current << std::endl;
    std::cout << "control_target:" << target << std::endl;
    if (prev_error_ * error <= 0) {
      integral_ = 0.0;
    }
    if (abs(error) < 0.001) {
      ConvertPoseToMsg(0.0, command);
    } else {
    integral_ += error * dt_;
      double derivative = (error - prev_error_) / dt_;
      double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
      prev_error_ = error;
      ConvertPoseToMsg(output, command);
    }

    return command;
  }
  void ResetPoseControl() {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }
  void ConvertPoseToMsg(double control_value, unitree_go::msg::DogControlCommand &command) {
    uint8_t modes[9] = {0, 0, 0, 0, 1, 0, 0, 0, 1};
    for (int i = 0; i < 9; ++i) {
      command.control_mode[i] = modes[i];
    }
    geometry_msgs::msg::Twist control_point;
    control_point.linear.x = 0.0;
    control_point.linear.y = 0.0;
    control_point.linear.z = control_value;
    control_point.angular.x = 0.0;
    control_point.angular.y = 0.0;
    control_point.angular.z = 0.0;
    command.point = control_point;
  }
  //*****************TrajectoryControl*************//
  unitree_go::msg::DogControlCommand
  TrajectoryControl(const PlanningResult &planning_result) {
    unitree_go::msg::DogControlCommand command;
    std::vector<unitree_go::msg::PathPoint> path;
    uint8_t modes[9] = {1, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 9; ++i) {
      command.control_mode[i] = modes[i];
    }

    for (int i = 0; i < 30; i++) {
            unitree_go::msg::PathPoint path_point;
      if (static_cast<int>(planning_result.size()) == 1) {
        path_point.t_from_start = last_command_.path_point.at(2).t_from_start;
        path_point.x = last_command_.path_point.at(4).x;
        path_point.y = last_command_.path_point.at(4).y;
        path_point.yaw = last_command_.path_point.at(4).yaw;
        path_point.vx = last_command_.path_point.at(4).vx;
        path_point.vy = last_command_.path_point.at(4).vy;
        path_point.vyaw = last_command_.path_point.at(4).vyaw;
        command.path_point[i] = path_point;
      } else{
        path_point.t_from_start = planning_result.at(i).t;
        path_point.x = planning_result.at(i).x;
        path_point.y = planning_result.at(i).y;
        path_point.yaw = planning_result.at(i).yaw;
        path_point.vx = planning_result.at(i).vx;
        path_point.vy = planning_result.at(i).vy;
        path_point.vyaw = planning_result.at(i).vyaw;
        command.path_point[i] = path_point;
        std::cout << "trajectory_s:" << planning_result.at(i).s << std::endl;
      std::cout << "trajectory_t:" << planning_result.at(i).t << std::endl;
      std::cout << "trajectory_x:" << planning_result.at(i).x << std::endl;
      std::cout << "trajectory_y:" << planning_result.at(i).y << std::endl;
      std::cout << "trajectory_yaw:" << planning_result.at(i).yaw << std::endl;
      std::cout << "trajectory_vx:" << planning_result.at(i).vx << std::endl;
      std::cout << "trajectory_vy:" << planning_result.at(i).vy << std::endl;
      std::cout << "trajectory_vyaw:" << planning_result.at(i).vyaw << std::endl;
      last_command_ = command;
      }
    }

    return command;
  }

private:
  double dt_{0.02};
  double kp_{0.1};
  double ki_{0.01};
  double kd_{0.001};
  double integral_{0.0};
  double prev_error_{0.0};
  unitree_go::msg::DogControlCommand last_command_;
};
} // namespace planning
} // namespace unitree