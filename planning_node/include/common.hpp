#pragma once
#include <vector>
#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"
namespace unitree {
namespace planning {
/************************data_type**************************/
struct RoutingPoint {
  double x;
  double y;
  double heading;
};
struct MapPoint {
  double x;
  double y;
  double hight;
};
struct StatePoint {
  double x;
  double y;
  double z;
  double heading;
  double v;
  bool isEqual(const StatePoint& other, double epsilon = 1e-6) const {
      return std::fabs(x - other.x) < epsilon &&
              std::fabs(y - other.y) < epsilon &&
              std::fabs(heading - other.heading) < epsilon;
  }
};
struct SearchArea {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};
struct PlanningPoint {
  double s;
  double t;
  double x;
  double y;
  double yaw;
  double vx;
  double vy;
  double vyaw;
};
using PlanningResult = std::vector<PlanningPoint>;
struct SearchIn {
  int next_heading_class; // 0: 和end_point同向， 1：左转90度，-1：右转90度
  StatePoint loc_point;
  StatePoint end_point;
  StatePoint map_origin_point;
  SearchArea search_area;
  std::vector<MapPoint> map_data;
  std::vector<RoutingPoint> ref_path;
  std::vector<RoutingPoint> left_path;
  std::vector<RoutingPoint> right_path;
};
struct SearchSegmentResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> kappa;
  std::vector<double> accumulated_s;
  int gear;  // 1:P, 2:R, 3:N, 4:D
};
/************************search_config**************************/
struct HybirdAStarConfig {
  double pos_resolution{0.06};
  double expand_radius{0.4}; // 因为是3个圆判断，dog长度为0.7，0.7/2/2
  int max_search_times{20000};
  int node_explore_size{6};
  double max_steer_angle{1.6}; // TODO(all) test
  double node_dt{0.5}; // TODO(all) test
  double max_steer_anglerate{1.8}; // TODO(all) test
  double search_step_s{0.1}; // TODO(all) test
  double wheel_base{1.0}; // TODO(all) test
  double finish_max_delta_theta{0.5};  // TODO(all) test
  double finish_max_delta_distance{0.1};   // TODO(all) test
  double cost_weight{1.0};
  double cost_steer{0.0};
  double cost_steerrate{0.0};
  int preiew_length{1};
  double cost_refline_h{30.0};
  double theta_resolution{0.1};
};
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}
inline double NormalizeAngle(const double angle) {
double a = std::fmod(angle + M_PI, 2.0 * M_PI);
if (a < 0.0) {
  a += (2.0 * M_PI);
}
return a - M_PI;
}
double convert_orientation_to_eular(const geometry_msgs::msg::Quaternion &quaternion) {
  double siny_cosp = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  double cosy_cosp = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return NormalizeAngle(atan2(siny_cosp, cosy_cosp));
}
} // namespace path
} // namespace planning