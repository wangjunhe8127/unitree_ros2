#include "rclcpp/rclcpp.hpp"

#include <string>
// rviz可视化
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/height_map.hpp"
using namespace std::chrono_literals;
class HightMapNode : public rclcpp::Node {
public:
  HightMapNode();

private:

  void sport_loc_callback(const unitree_go::msg::SportModeState::ConstSharedPtr data);
  void world_loc_callback(const nav_msgs::msg::Odometry::ConstSharedPtr data);
  void hight_map_callback(const unitree_go::msg::HeightMap::ConstSharedPtr msg);
  void convert_cloudpoints();
  void pubrviz();

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_loc_suber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr world_loc_suber_;
  rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr hight_map_suber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rivz_puber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_hight_map_puber_;
  
  nav_msgs::msg::Odometry sport_loc_;
  nav_msgs::msg::Odometry world_loc_;

  tf2::Transform transform_local_;
  tf2::Transform transform_world_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_hight_map_;
};