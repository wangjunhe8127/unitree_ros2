
#include "rclcpp/rclcpp.hpp"
#include "local_planner_node.hpp"
using std::placeholders::_1;

  LocalPlannerNode::LocalPlannerNode() : Node("LocalPlannerNode")
  {
    loc_suber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        loc_topic_, 10, std::bind(&LocalPlannerNode::loc_callback, this, _1));
    waypoint_suber_ = this->create_subscription<geometry_msgs::msg::Point>(
        waypoint_topic_, 10, std::bind(&LocalPlannerNode::waypoint_callback, this, _1));
    nav_status_suber_ = this->create_subscription<std_msgs::msg::Bool>(
        loc_topic_, 10, std::bind(&LocalPlannerNode::nav_status_callback, this, _1));
    pointcloud_suber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
        pointcloud_topic_, 10, std::bind(&LocalPlannerNode::pointcloud_callback, this, _1));

    path_puber_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
      run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(ros_ms),
                              std::bind(&LocalPlannerNode::run_step, this));
  }
void LocalPlannerNode::run_step(){
  
}
  void LocalPlannerNode::waypoint_callback(const geometry_msgs::msg::Point::ConstSharedPtr data)
  {
    goal_point_.x = data->x;
    goal_point_.y = data->y;
    goal_point_.heading = data->z;
};
void LocalPlannerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr data) {
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

      float dis = sqrt((pointX - loc_point_.x) * (pointX - loc_point_.x) + (pointY - loc_point_.y) * (pointY - loc_point_.y));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
void LocalPlannerNode::loc_callback(const geometry_msgs::msg::Pose::ConstSharedPtr data) {
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  loc_point_.x = data->position.x;
  loc_point_.y = data->position.y;
  loc_point_.z = data->position.z;
  loc_point_.heading = unitree::planning::convert_orientation_to_eular(data->orientation);
}
void LocalPlannerNode::nav_status_callback(const std_msgs::msg::Bool::ConstSharedPtr data) {
  nav_status_ = data->data;
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals
    rclcpp::spin(std::make_shared<StateConvertNode>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}
