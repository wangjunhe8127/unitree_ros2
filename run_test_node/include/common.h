#include <Eigen/Core>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

double convert_orientation_to_eular(geometry_msgs::msg::Pose &pose) {
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
  return eulerAngle(0);
}
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt) :
        kp_(kp), ki_(ki), kd_(kd), dt_(dt) {}
    double compute(double error) {
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }
private:
    double dt_{0.02};
    double kp_{0.1};
    double ki_{0.01};
    double kd_{0.001};
    double integral_{0.0};
    double prev_error_{0.0};
};