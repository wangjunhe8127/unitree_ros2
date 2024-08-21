#include <vector>
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
  double heigh;
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
} // namespace path
} // namespace planning