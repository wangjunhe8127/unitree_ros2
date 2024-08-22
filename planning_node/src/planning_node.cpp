
#include "planning_node.hpp"
namespace unitree {
namespace planning {
using std::placeholders::_1;
PlanningNode::PlanningNode() : Node("PlanningNode") {
  state_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      state_topic_name_, 10,
      std::bind(&PlanningNode::state_callback, this, _1));
  map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
      map_topic_name_, 10, std::bind(&PlanningNode::map_callback, this, _1));
  routing_suber_ = this->create_subscription<unitree_go::msg::Routing>(
      routing_topic_name_, 10,
      std::bind(&PlanningNode::routing_callback, this, _1));
  run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&PlanningNode::run_step, this));
  search_core_ = std::make_shared<SearchCore>();
}
void PlanningNode::run_step() {
  if (!state_receive_ || !map_receive_ || !routing_receive_) {
    return;
  }
  search_result_.clear();
  search_core_->Init(search_in_);
  // if (reach_end_flag_ && SameEndPoint()) {
  //   // PID执行目标旋转角度
  //   res = motion_core->PoseControl(); // 输入当前状态，目标状态，输出控制信息
  // } else {
  //   reach_end_flag_,目标状态 = ReachEnd(); //会延时一帧
  //   search_result_.clear();
  //   search_core_->Init(search_in_);
  //   if (search_core_->Process(search_result_)) {
  //     res = motion_core->TrajectoryControl(search_result_);
  //   }
  // }

}
void PlanningNode::state_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  state_receive_ = true;
  search_in_.loc_point.x = data->pose.position.x;
  search_in_.loc_point.y = data->pose.position.y;
  double heading = convert_orientation_to_eular(data->pose.orientation);
  search_in_.loc_point.heading = heading;
}
void PlanningNode::map_callback(unitree_go::msg::HeightMap::SharedPtr data) {
  // receive_flag
  map_receive_ = true;
  // map_info
  int width = data->width;
  int height = data->height;
  double resolution = data->resolution;
  double originX = data->origin[0];
  double originY = data->origin[1];

  std::cout << "width:" << width << std::endl;
  std::cout << "height:" << height << std::endl;
  std::cout << "resolution:" << resolution << std::endl;
  search_in_.map_origin_point.x = originX;
  search_in_.map_origin_point.y = originY;
  // member variable
  search_in_.map_data.clear();
  search_in_.map_data.reserve(height * width);
  // map point
  MapPoint point;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      point.hight = data->data[ix + width * iy];
      if (point.hight == 1.0e9) {
        continue;
      }
      point.x = ix * resolution + originX;
      point.y = iy * resolution + originY;
      search_in_.map_data.emplace_back(point);
    }
  }
  search_in_.search_area.min_x = originX;
  search_in_.search_area.min_y = originY;
  search_in_.search_area.max_x = width * resolution + originX;
  search_in_.search_area.max_y = height * resolution + originY;
  std::cout << "min_x:" << search_in_.search_area.min_x
            << " max_x:" << search_in_.search_area.max_x
            << " min_y:" << search_in_.search_area.min_y
            << " max_y: " << search_in_.search_area.max_y << std::endl;
}
void PlanningNode::routing_callback(unitree_go::msg::Routing::SharedPtr data) {
  routing_receive_ = true;
  search_in_.ref_path.clear();
  search_in_.left_path.clear();
  search_in_.right_path.clear();
  search_in_.ref_path.reserve(static_cast<int>(data->ref_line.size()));
  search_in_.left_path.reserve(static_cast<int>(data->left_line.size()));
  search_in_.right_path.reserve(static_cast<int>(data->right_line.size()));
  RoutingPoint point;
  std::cout << "ref_line.size: " << data->ref_line.size() << std::endl;
  for (int i = 0; i < static_cast<int>(data->ref_line.size()); i++) {
    point.x = data->ref_line.at(i).x;
    point.y = data->ref_line.at(i).y;
    point.heading = data->ref_line.at(i).z;
    search_in_.ref_path.emplace_back(point);
  }
  for (int i = 0; i < static_cast<int>(data->left_line.size()); i++) {
    point.x = data->left_line.at(i).x;
    point.y = data->left_line.at(i).y;
    std::cout << "left_x:" << point.x << std::endl;
    std::cout << "left_y:" << point.y << std::endl;
    point.heading = data->left_line.at(i).z;
    search_in_.left_path.emplace_back(point);
  }
  for (int i = 0; i < static_cast<int>(data->right_line.size()); i++) {
    point.x = data->right_line.at(i).x;
    point.y = data->right_line.at(i).y;
    std::cout << "right_x:" << point.x << std::endl;
    std::cout << "right_y:" << point.y << std::endl;
    point.heading = data->right_line.at(i).z;
    search_in_.right_path.emplace_back(point);
  }
  search_in_.end_point.x = data->end_point.x;
  search_in_.end_point.y = data->end_point.y;
  search_in_.end_point.heading = data->end_point.z;
  std::cout << "end_x:" << data->end_point.x << std::endl;
  std::cout << "end_y:" << data->end_point.y << std::endl;
};
} // namespace planning
} // namespace unitree
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unitree::planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}