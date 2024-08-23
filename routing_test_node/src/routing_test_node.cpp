
#include "routing_test_node.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
RoutingTesetNode::RoutingTesetNode() : Node("RoutingTesetNode") {
  loc_suber_ = this->create_subscription<unitree_go::msg::DogReportCommon>(
      loc_topic_name_, 10,
      std::bind(&RoutingTesetNode::loc_callback, this, _1));
  routing_puber_ =
      this->create_publisher<unitree_go::msg::Routing>(routing_topic_name_, 10);
  run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&RoutingTesetNode::run_step, this));
}
void RoutingTesetNode::run_step() {
  if (receive_loc_) {
  }
}
void RoutingTesetNode::loc_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  double boundary_width = 0.5;
  double spacing = 0.06;
  double end_s = 6.0;
  int numPoints = static_cast<int>(end_s / spacing);
  double x_ = data->pose.position.x;
  double y_ = data->pose.position.y;
  double heading =
      unitree::planning::convert_orientation_to_eular(data->pose.orientation);
  if (!receive_loc_) {
    receive_loc_ = true;
    init_heading_ = heading;
    init_x_ = x_;
    init_y_ = x_;
    for (int i = 0; i < numPoints; ++i) {
      double dx = i * spacing * cos(heading);
      double dy = i * spacing * sin(heading);
      std::pair<double, double> refPoint = {x_ + dx, y_ + dy};
      // 生成左右边界点
      std::pair<double, double> leftPoint = {
          refPoint.first - boundary_width * sin(heading),
          refPoint.second + boundary_width * cos(heading)};
      std::pair<double, double> rightPoint = {
          refPoint.first + boundary_width * sin(heading),
          refPoint.second - boundary_width * cos(heading)};
      ref_.push_back(refPoint);
      left_.push_back(leftPoint);
      right_.push_back(rightPoint);
    }
  }
  double minDistance = std::numeric_limits<double>::max();
  int index = 0;
  for (int i = 0; i < ref_.size(); i++) {
    double distance = std::hypot(ref_[i].first - x_, ref_[i].second - y_);
    if (distance < minDistance) {
      minDistance = distance;
      index = i;
    }
  }
  int search_unit_num = static_cast<int>(7.62 / 2.0 / spacing);
  std::vector<std::pair<double, double>> newRef(
      ref_.begin() + index,
      std::min(ref_.begin() + index + search_unit_num, ref_.end()));
  std::vector<std::pair<double, double>> newLeft(
      left_.begin() + index,
      std::min(left_.begin() + index + search_unit_num, left_.end()));
  std::vector<std::pair<double, double>> newRight(
      right_.begin() + index,
      std::min(right_.begin() + index + search_unit_num, right_.end()));
  ref_ = newRef;
  left_ = newLeft;
  right_ = newRight;
  unitree_go::msg::Routing routing;
  for (int i = 0; i < ref_.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = ref_.at(i).first;
    point.y = ref_.at(i).second;
    point.z = init_heading_;
    routing.ref_line.push_back(point);
  }
  for (int i = 0; i < left_.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = left_.at(i).first;
    point.y = left_.at(i).second;
    point.z = init_heading_;
    routing.left_line.push_back(point);
  }
  for (int i = 0; i < right_.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = right_.at(i).first;
    point.y = right_.at(i).second;
    point.z = init_heading_;
    routing.right_line.push_back(point);
  }
  geometry_msgs::msg::Point point;
  point.x = end_s * cos(init_heading_) + init_x_;
  point.y = end_s * sin(init_heading_) + init_y_;
  point.z = init_heading_;
  routing.end_point = point;
  routing.next_heading_class = 1;
  routing_puber_->publish(routing);

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                           // Initialize rclcpp
  rclcpp::spin(std::make_shared<RoutingTesetNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
