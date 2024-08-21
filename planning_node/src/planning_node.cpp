
#include "planning_node.hpp"
namespace unitree {
namespace planning {
PlanningNode::PlanningNode() : Node("PlanningNode") {
  state_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      state_topic_name_, 10, std::bind(&PlanningNode::state_callback, this, _1));
  map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
      map_topic_name_, 10, std::bind(&PlanningNode::map_callback, this, _1));
  // routing_suber_ = this->create_subscription<unitree_go::msg::Routing>(
  //     routing_topic_name_, 10,
  //     std::bind(&PlanningNode::routing_callback, this, _1));
  run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PlanningNode::run_step, this));

}
void PlanningNode::run_step() {
  if (!state_receive_ || !map_receive_ || !routing_receive_) {
    return;
  }
  search_result_.clear();
  search_core_->Init(search_in_);
}
void PlanningNode::state_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  state_receive_ = true;
  search_in_.state_data.x = data->pose.position.x;
  search_in_.state_data.y = data->pose.position.y;
  search_in_.state_data.yaw = data->pose.position.yaw;
};
void PlanningNode::map_callback(unitree_go::msg::HeightMap::SharedPtr data) {
  // receive_flag
  map_receive_ = true;
  // map_info
  int width = data->width;
  int height = data->height;
  double resolution = data->resolution;
  double originX = data->origin[0];
  double originY = data->origin[1];
  search_in_.map_origin_point.x = originX;
  search_in_.map_origin_point.y = originY;
  // member variable
  search_in_.map_data.clear();
  search_in_.map_data.reserve(height * width);
  // map point
  std::array<float, 3> point;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      point[2] = msg->data[ix + width * iy];
      if (point[2] == 1.0e9) {
        continue;
      }
      point[0] = ix * resolution + originX;
      point[1] = iy * resolution + originY;
      search_in_.map_data.push_back(point);
    }
  }
};
void PlanningNode::routing_callback(unitree_go::msg::Routing::SharedPtr data){

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
} // namespace path
} // namespace planning