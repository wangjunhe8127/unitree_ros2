
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
  planning_puber_ = this->create_publisher<unitree_go::msg::DogControlCommand>(
      planning_topic_name_, 10);
  run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_time_), std::bind(&PlanningNode::run_step, this));
  search_core_ = std::make_shared<SearchCore>();
  motion_core = std::make_shared<MotionCore>(0.1,0.01,0.001,1.0/timer_time_);
}
void PlanningNode::run_step() {
  if (!state_receive_ || !map_receive_ || !routing_receive_) {
    return;
  }
  if (routing_finish_) {
    std::cout << "Dog arrive routing end." << std::endl;
    return;
  }
  // search_result_.clear();
  // search_core_->Init(search_in_);
  unitree_go::msg::DogControlCommand command;
  if (reach_end_flag_ && SameEndPoint()) {
    // PID执行目标旋转角度
    command =
        motion_core->PoseControl(search_in_.loc_point.heading, target_heading_);
  } else {
    motion_core->ResetPoseControl();
    reach_end_flag_ = CheckReachEnd(); //会延时一帧
    search_core_->Init(search_in_);
    if (search_core_->Process(planning_result_)) {
      // 执行优化轨迹控制
      command = motion_core->TrajectoryControl(planning_result_);
    }
  }
  std::cout << "end_record" << std::endl;
  planning_puber_->publish(command);
  last_end_point_ = search_in_.end_point;
}
bool PlanningNode::CheckReachEnd() {
  double dx = search_in_.loc_point.x - search_in_.end_point.x;
  double dy = search_in_.loc_point.y - search_in_.end_point.y;
  double distance = std::hypot(dx, dy);
  std::cout << "distance:" << distance << std::endl;
  if (distance < reach_end_th_) {
    target_heading_ = search_in_.end_point.heading;
    target_heading_ =
        target_heading_ + M_PI / 2.0 * search_in_.next_heading_class;
    return true;
  }
  return false;
}
bool PlanningNode::SameEndPoint() {
  if (last_end_point_.isEqual(search_in_.end_point)) {
    return true;
  }
  return false;
}
void PlanningNode::state_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  state_receive_ = true;
  search_in_.loc_point.x = data->pose.position.x;
  search_in_.loc_point.y = data->pose.position.y;
  search_in_.loc_point.v = std::hypot(data->speed.vx, data->speed.vy);
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
  if (data->finish == 1) {
    routing_finish_ = true;
    return;
  }
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
  search_in_.next_heading_class = data->next_heading_class;
};
} // namespace planning
} // namespace unitree
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unitree::planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}