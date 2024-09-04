
#include "local_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

LocalPlannerNode::LocalPlannerNode() : Node("LocalPlannerNode") {
  loc_suber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      loc_topic_, 10, std::bind(&LocalPlannerNode::loc_callback, this, _1));
  waypoint_suber_ = this->create_subscription<geometry_msgs::msg::Point>(
      waypoint_topic_, 10,
      std::bind(&LocalPlannerNode::waypoint_callback, this, _1));
  nav_status_suber_ = this->create_subscription<std_msgs::msg::Bool>(
      loc_topic_, 10,
      std::bind(&LocalPlannerNode::nav_status_callback, this, _1));
  pointcloud_suber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      pointcloud_topic_, 10,
      std::bind(&LocalPlannerNode::pointcloud_callback, this, _1));

  path_puber_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(ros_ms),
                              std::bind(&LocalPlannerNode::run_step, this));
  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  path_planner_ = std::make_shared<PathPlanCore>();
  path_planner_->init();
}
void LocalPlannerNode::run_step() {
  if (receive_loc_ && receive_goal_ && receive_pointcloud_) {
    bool status =
        path_planner_->process(loc_point_, goal_point_, pointcloud_, nav_path_);
    path_puber_->publish(nav_path_);
  }
}
void LocalPlannerNode::waypoint_callback(
    const geometry_msgs::msg::Point::ConstSharedPtr data) {
  receive_goal_ = true;
  goal_point_.x = data->x;
  goal_point_.y = data->y;
  goal_point_.heading = data->z;
};
void LocalPlannerNode::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr data) {
  receive_pointcloud_ = true;
  laserCloud->clear();
  pcl::fromROSMsg(*data, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - loc_point_.x) * (pointX - loc_point_.x) +
                     (pointY - loc_point_.y) * (pointY - loc_point_.y));
    if (dis < adjacentRange) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      laserCloudCrop->push_back(point);
    }
  }

  pointcloud_->clear();
  laserDwzFilter.setInputCloud(laserCloudCrop);
  laserDwzFilter.filter(*pointcloud_);
}
void LocalPlannerNode::loc_callback(
    const geometry_msgs::msg::Pose::ConstSharedPtr data) {
  receive_loc_ = true;
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  loc_point_.x = data->position.x;
  loc_point_.y = data->position.y;
  loc_point_.z = data->position.z;
  loc_point_.heading =
      unitree::planning::convert_orientation_to_eular(data->orientation);
}
void LocalPlannerNode::nav_status_callback(
    const std_msgs::msg::Bool::ConstSharedPtr data) {
  receive_nav_status_ = true;
  nav_status_ = data->data;
}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);            // Initialize rclcpp
  rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send
                                       // sport request in time intervals
  rclcpp::spin(std::make_shared<StateConvertNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
