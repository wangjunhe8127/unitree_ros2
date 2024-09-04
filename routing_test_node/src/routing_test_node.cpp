
#include "routing_test_node.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
RoutingTesetNode::RoutingTesetNode()
    : Node("RoutingTesetNode"), tf_buffer(std::make_shared<rclcpp::Clock>()),
      tf_listener(tf_buffer) {
  waypoint_puber_ =
      this->create_publisher<geometry_msgs::msg::Point>(waypoint_topic_name_, 10);
  loc_puber_ =
      this->create_publisher<geometry_msgs::msg::Pose>(loc_topic_name_, 10);
  nav_status_puber_ =
      this->create_publisher<std_msgs::msg::Bool>(nav_status_topic_name_, 10);
  run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&RoutingTesetNode::run_step, this));
  load_waypoints("/home/unitree/code/unitree_ros2/routing_test_node/waypoints.txt");
}
void RoutingTesetNode::run_step() {
  loc_callback();
  if (receive_loc_) {
    loc_puber_->publish(loc_pose_);
    send_waypoint();
  }
}
bool RoutingTesetNode::load_waypoints(const std::string &waypoint_path) {
  std::ifstream file(waypoint_path);
  if (!file.is_open()) {
    std::cerr << "无法打开文件：" << waypoint_path << std::endl;
    return false;
  }
  std::string line;
  double x, y, yaw;
  waypoints_.clear();
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
    std::cout << "读取的数据: " << x << ", " << y << ", " << yaw << std::endl;
    waypoints_.push_back(point);
  }
  // 关闭文件
  file.close();
  return true;
}
void RoutingTesetNode::loc_callback() {
  try {
    // 从map到lidar获取变换
    geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer.lookupTransform("lidar", "map", tf2::TimePointZero);
    // 输出接收到的变换信息
    RCLCPP_INFO(this->get_logger(), "Received translation: (%.2f, %.2f, %.2f)",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "Received rotation: (%.2f, %.2f, %.2f, %.2f)",
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
    loc_pose_.position.x = transformStamped.transform.translation.x;
    loc_pose_.position.y = transformStamped.transform.translation.y;
    loc_pose_.position.z = transformStamped.transform.translation.z;
    loc_pose_.orientation.x = transformStamped.transform.rotation.x;
    loc_pose_.orientation.y = transformStamped.transform.rotation.y;
    loc_pose_.orientation.z = transformStamped.transform.rotation.z;
    loc_pose_.orientation.w = transformStamped.transform.rotation.w;
    receive_loc_ = true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
  }
}
bool RoutingTesetNode::check_waypoint_finish() {
  if (arrive_end_) {
    nav_status_.data = false;
    nav_status_puber_->publish(nav_status_);
    return false;
  }
  double loc_yaw, diff_dis, diff_yaw;
  diff_dis = std::hypot(loc_pose_.position.x - waypoints_.at(waypoint_idx_).at(0),
                        loc_pose_.position.y - waypoints_.at(waypoint_idx_).at(1));
  loc_yaw = unitree::planning::convert_orientation_to_eular(loc_pose_.orientation);
  diff_yaw = abs(loc_yaw - waypoints_.at(waypoint_idx_).at(2));
  if (diff_dis < diff_dis_th_ && diff_yaw < diff_yaw_th_) {
    if (waypoint_idx_ == static_cast<int>(waypoints_.size() - 1)) {
      arrive_end_ = true;
      nav_status_.data = false;
      nav_status_puber_->publish(nav_status_);
      return false;
    }
    nav_status_.data = true;
    nav_status_puber_->publish(nav_status_);
    waypoint_idx_++;
  } else {
    nav_status_.data = true;
    nav_status_puber_->publish(nav_status_);
  }
  return true;
}
void RoutingTesetNode::send_waypoint() {
  if (check_waypoint_finish()) {
    waypoint_.x = waypoints_.at(waypoint_idx_).at(0);
    waypoint_.y = waypoints_.at(waypoint_idx_).at(1);
    waypoint_.z = waypoints_.at(waypoint_idx_).at(2);
    waypoint_puber_->publish(waypoint_);
  }
}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                           // Initialize rclcpp
  rclcpp::spin(std::make_shared<RoutingTesetNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
