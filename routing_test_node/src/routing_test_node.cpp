
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
bool RoutingTesetNode::LoadWayPoints(const std::string &waypoint_path) {
  std::ifstream file(waypoint_path);
  if (!file.is_open()) {
    std::cerr << "无法打开文件：" << filename << std::endl;
    return false;
  }
  std::string line;
  std::vector<std::vector<double>> waypoints;
  double x, y, yaw;
  // 逐行读取数据
  while (getline(file, line)) {
    std::vector<double> point;
    std::istringstream iss(line); // 将读取的行转换为字符串流
    std::string value;
    std::getline(iss, value, ',');
    x = std::stod(value);
    point.push_back(x);
    // 读取第二个数据
    std::getline(iss, value, ',');
    y = std::stod(value); // 转换为double
    point.push_back(y);
    // 读取第三个数据
    std::getline(iss, value);
    yaw = std::stod(value); // 转换为double
    point.push_back(yaw);
    // 输出读取的数据
    std::cout << "读取的数据: " << num1 << ", " << num2 << ", " << num3
              << std::endl;
    waypoints.push_back(point);
  }
  // 关闭文件
  file.close();
  return true;
}
void RoutingTesetNode::loc_callback(
    unitree_go::msg::DogReportCommon::SharedPtr data) {
  unitree_go::msg::Routing routing;
  double boundary_width = 0.5;
  double spacing = 0.06;
  double end_s = 6.0;
  int numPoints = static_cast<int>(end_s / spacing);
  double x_ = data->pose.position.x;
  double y_ = data->pose.position.y;
  double heading =
      unitree::planning::convert_orientation_to_eular(data->pose.orientation);
  std::cout << "heading: " << heading << std::endl;
  std::cout <<"rececive";
  if (!receive_loc_) {
    receive_loc_ = true;
    init_heading_ = heading;
    init_x_ = x_;
    init_y_ = y_;
    std::cout << init_x_ << std::endl;
    std::cout << init_y_ << std::endl;
    end_point_.x = end_s * cos(init_heading_) + init_x_;
    end_point_.y = end_s * sin(init_heading_) + init_y_;
    end_point_.z = init_heading_;
    std::cout << "end_x:" << end_point_.x << std::endl;
    std::cout << "end_y:" << end_point_.y << std::endl;
    std::cout << end_point_.x << std::endl;
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
  } else {
    double dx = x_ - end_point_.x;
    double dy = y_ - end_point_.y;
    double dyaw = heading - (end_point_.z + M_PI / 2.0);
    std::cout << "end_heading:" << end_point_.z <<std::endl;
    std::cout << "target_heading: " << M_PI / 2.0 <<std::endl;
    std::cout << "routing_end_dis:" << std::hypot(dx, dy) << std::endl;
    std::cout << "routing_end_yaw:" << dyaw  << std::endl;
    if (std::hypot(dx, dy) < dis_finish_th_ && abs(dyaw) < yaw_finish_th_) {
      std::cout << "end" <<std::endl;
      routing.finish = 1;
      routing_puber_->publish(routing);
      return;
    }
  }
    std::cout << "rr:" << std::endl;

  double minDistance = std::numeric_limits<double>::max();
  int index = 0;
  for (int i = 0; i < ref_.size(); i++) {
    double distance = std::hypot(ref_[i].first - x_, ref_[i].second - y_);
    if (distance < minDistance) {
      minDistance = distance;
      index = i;
    }
  }
  int search_unit_num = static_cast<int>(3.0 / spacing);
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
std::cout << "tt:" << std::endl;
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
  routing.end_point = end_point_;
  routing.next_heading_class = 1;
  routing_puber_->publish(routing);
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                           // Initialize rclcpp
  rclcpp::spin(std::make_shared<RoutingTesetNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
