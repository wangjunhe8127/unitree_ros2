#include <vector>
namespace planning {
namespace path {
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
  StatePoint state_data;
  std::vector<MapPoint> map_data;
  std::vector<RoutingPoint> ref_path;
  std::vector<RoutingPoint> left_path;
  std::vector<RoutingPoint> right_path;
};
using SearchOut = std::vector<StatePoint>;
} // namespace path
} // namespace planning