
#include "search_node.hpp"
namespace planning {
namespace path {
SearchNode::SearchNode() : Node("SearchNode") {
  state_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      state_topic_name_, 10, std::bind(&SearchNode::state_callback, this, _1));
  map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
      map_topic_name_, 10, std::bind(&SearchNode::map_callback, this, _1));
  // routing_suber_ = this->create_subscription<unitree_go::msg::Routing>(
  //     routing_topic_name_, 10,
  //     std::bind(&SearchNode::routing_callback, this, _1));
}
void SearchNode::run_step() {
  if (!state_receive_ || !map_receive_ || !routing_receive_) {
    return;
  }
  search_result_.clear();
  search_flag_ = search_core_->Process(search_in_, search_result_);
}
void SearchNode::state_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  state_receive_ = true;
  search_in_.state_data.x = data->pose.position.x;
  search_in_.state_data.y = data->pose.position.y;
  search_in_.state_data.yaw = data->pose.position.yaw;
};
void SearchNode::map_callback(unitree_go::msg::HeightMap::SharedPtr data) {
  // receive_flag
  map_receive_ = true;
  // map_info
  int width = data->width;
  int height = data->height;
  double resolution = data->resolution;
  double originX = data->origin[0];
  double originY = data->origin[1];
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
void SearchNode::routing_callback(unitree_go::msg::Routing::SharedPtr data){

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::spin(std::make_shared<SearchNode>());
  rclcpp::shutdown();
  return 0;
}
} // namespace path
} // namespace planning