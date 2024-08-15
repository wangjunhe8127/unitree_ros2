#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_report_common.hpp"
#include "unitree_go/msg/height_map.hpp"
#include <string>
// rviz可视化
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;
class HightMapNode : public rclcpp::Node {
public:
  HightMapNode();

private:
  void callback(unitree_go::msg::HeightMap::SharedPtr data);
  void pubrviz(const std::vector<std::array<float, 3>> &cloud);
  std::string sub_topic_name_ = "/utlidar/height_map_array";
  std::string rviz_topic_name_ = "/rviz/hight_map";
  rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr suber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rivz_puber_;
};