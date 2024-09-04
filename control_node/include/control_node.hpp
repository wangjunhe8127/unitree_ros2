#include <string>
#include "rclcpp/rclcpp.hpp"
#include "common.hpp"
#include "unitree_api/msg/request.hpp"
#include "ros2_sport_client.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include "control_deva.hpp"

class ControlNode : public rclcpp::Node{
 public:
  ControlNode();

 private:
  // ros ms
  int ros_ms{20};
  // topic name
  std::string loc_topic_ = "/routing/loc";
  std::string path_topic_ = "/planning/path";
  std::string stop_topic_ = "/routing/nav_status";
  std::string control_topic_ = "/api/sport/request";
  // suber/puber
  rclcpp::TimerBase::SharedPtr run_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr loc_suber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_suber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_suber_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr control_puber_;
  // callback
  void run_step();
  void loc_callback(const geometry_msgs::msg::Pose::ConstSharedPtr data);
  void path_callback(const nav_msgs::msg::Path::ConstSharedPtr data);
  void stop_callback(const std_msgs::msg::Bool::ConstSharedPtr data);
  // var
  bool receive_loc_{false};
  bool receive_path_{false};
  bool receive_stop_{false};
  bool stop_{false};
  nav_msgs::msg::Path path_;
  unitree::planning::StatePoint loc_point_;
  // api
  unitree_api::msg::Request req;
  // deva
  std::shared_ptr<Controller> controller_;
};