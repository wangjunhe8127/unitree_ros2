#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}
inline double QuaternionToHeading(const double qw, const double qx,
                                const double qy, const double qz) {
    // the heading is zero when the car is pointing East.
    double siny_cosp = +2.0 * (qw * qz + qx * qy);
    double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
    return NormalizeAngle(atan2(siny_cosp, cosy_cosp));
}

double convert_orientation_to_eular(geometry_msgs::msg::Quaternion &quaternion) {
  double heading = QuaternionToHeading(quaternion.w, quaternion.x,quaternion.y,quaternion.z);
  return heading;
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