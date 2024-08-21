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
};
struct MapPoint {
  double x;
  double y;
  double hight;
};
struct StatePoint {
  double x;
  double y;
  double yaw;
};
struct SearchIn {
  StatePoint loc_point;
  StatePoint map_origin_point;
  std::vector<MapPoint> map_data;
  std::vector<RoutingPoint> ref_path;
  std::vector<RoutingPoint> left_path;
  std::vector<RoutingPoint> right_path;
};
using SearchOut = std::vector<StatePoint>;
/************************search_config**************************/
struct HybirdAStarConfig {
  double pos_resolution{0.25};
  double theta_resolution{0.1};
  double expand_radius{1.0};
};
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