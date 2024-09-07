
#include "routing_test_node.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
RoutingTesetNode::RoutingTesetNode()
    : Node("RoutingTesetNode"), tf_buffer(std::make_shared<rclcpp::Clock>()),
      tf_listener(tf_buffer) {
  waypoint_puber_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic_name_, 10);
  loc_puber_ =
      this->create_publisher<nav_msgs::msg::Odometry>(loc_topic_name_, 10);
  nav_status_puber_ =
      this->create_publisher<std_msgs::msg::Bool>(nav_status_topic_name_, 10);
  run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&RoutingTesetNode::run_step, this));
  r_puber_ = this->create_publisher<unitree_go::msg::DogControlCommand>(
      r_control_topic_name_, 10);
  load_waypoints("/home/unitree/code/unitree_ros2/routing_test_node/waypoints.txt");
  motion_core = std::make_shared<unitree::planning::MotionCore>(0.1,0.01,0.001,1.0/50);
}
void RoutingTesetNode::run_step() {
  loc_callback();
  if (receive_loc_) {
    loc_puber_->publish(loc_pose_);
    send_waypoint();
    control_r();
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
        tf_buffer.lookupTransform("map", "body", tf2::TimePointZero);
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
    loc_pose_.header.stamp = this->get_clock()->now();
    loc_pose_.header.frame_id = "map";
    loc_pose_.pose.pose.position.x = transformStamped.transform.translation.x;
    loc_pose_.pose.pose.position.y = transformStamped.transform.translation.y;
    loc_pose_.pose.pose.position.z = transformStamped.transform.translation.z;
    loc_pose_.pose.pose.orientation.x = transformStamped.transform.rotation.x;
    loc_pose_.pose.pose.orientation.y = transformStamped.transform.rotation.y;
    loc_pose_.pose.pose.orientation.z = transformStamped.transform.rotation.z;
    loc_pose_.pose.pose.orientation.w = transformStamped.transform.rotation.w;
    receive_loc_ = true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
  }
}
void RoutingTesetNode::control_r() {
  if (arrive_end_) {
    if (waypoint_idx_ != static_cast<int>(waypoints_.size() - 1)) {
      double loc_yaw = unitree::planning::convert_orientation_to_eular(loc_pose_.pose.pose.orientation);
      double diff_yaw = abs(loc_yaw - waypoints_.at(waypoint_idx_).at(2));
      if (diff_yaw < diff_yaw_th_) {
        waypoint_idx_++;
        motion_core->ResetPoseControl();
        arrive_end_ = false;
      } else {
        command_ = motion_core->PoseControl(loc_yaw, waypoints_.at(waypoint_idx_).at(2));
        r_puber_->publish(command_);
      }
    }
      
  }
}

bool RoutingTesetNode::check_waypoint_finish() {
  if (arrive_end_) {
    nav_status_.data = false;
    nav_status_puber_->publish(nav_status_);
    return false;
  }
  double diff_dis;
  diff_dis = std::hypot(loc_pose_.pose.pose.position.x - waypoints_.at(waypoint_idx_).at(0),
                        loc_pose_.pose.pose.position.y - waypoints_.at(waypoint_idx_).at(1));
  std::cout << "routing_dis: " <<diff_dis << std::endl;
  if (diff_dis < diff_dis_th_ ) {
      // arrive_end_ = true; //默认不使用单独旋转
      nav_status_.data = false;
      nav_status_puber_->publish(nav_status_);
      waypoint_idx_++;
      return false;
    }
    nav_status_.data = true;
    nav_status_puber_->publish(nav_status_);
  return true;
}
void RoutingTesetNode::send_waypoint() {
  if (check_waypoint_finish()) {
    waypoint_.header.stamp = this->get_clock()->now();
    // waypoint_.header.frame_id = "map";
    waypoint_.point.x = waypoints_.at(waypoint_idx_).at(0);
    waypoint_.point.y = waypoints_.at(waypoint_idx_).at(1);
    waypoint_.point.z = waypoints_.at(waypoint_idx_).at(2);
    waypoint_puber_->publish(waypoint_);
  }
}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                           // Initialize rclcpp
  rclcpp::spin(std::make_shared<RoutingTesetNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
