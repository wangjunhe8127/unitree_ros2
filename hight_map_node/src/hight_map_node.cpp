
#include "hight_map_node.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

HightMapNode::HightMapNode() : Node("HightMapNode") {
  suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
      sub_topic_name_, 10, std::bind(&HightMapNode::callback, this, _1));
  rivz_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      rviz_topic_name_, 10);
}
void HightMapNode::callback(unitree_go::msg::HeightMap::SharedPtr msg) {
  int width = msg->width;
  int height = msg->height;
  float resolution = msg->resolution;
  float originX = msg->origin[0];
  float originY = msg->origin[1];
  std::vector<std::array<float, 3>> cloud;
  int index;
  std::array<float, 3> pt;
  // std::cout<< "width: " << width <<std::endl;
  // std::cout<< "height: " << height <<std::endl;
  // std::cout<< "originX: " << originX <<std::endl;
  // std::cout<< "originY: " << originY <<std::endl;
  // std::cout<< "resolution: " << resolution <<std::endl;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      index = ix + width * iy;
      pt[2] = msg->data[index];
      if (pt[2] == 1.0e9) { // skip empty cell value which is set to 1.0e9
        continue;
      }
      pt[0] = ix * resolution + originX;
      pt[1] = iy * resolution + originY;
      cloud.push_back(pt);
      // std::cout<<"point: " << pt[0] << " " << pt[1] << " " << pt[2] <<std::endl;
    }
  }
  pubrviz(cloud);
}
void HightMapNode::pubrviz(const std::vector<std::array<float, 3>> &cloud) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.height = 1;
        msg.width = static_cast<int>(cloud.size());
        msg.fields.resize(3);
        msg.fields[0].name = "x";
        msg.fields[1].name = "y";
        msg.fields[2].name = "z";
        for (int i = 0; i < 3; ++i) {
            msg.fields[i].offset = i * sizeof(float);
            msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[i].count = 1;
        }

        msg.is_bigendian = false;
        msg.point_step = 3 * sizeof(float);
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);
        msg.is_dense = true;

        sensor_msgs::PointCloud2Iterator<float> x_iter(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iter(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iter(msg, "z");
        for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
            *x_iter = cloud.at(i)[0];
            *y_iter = cloud.at(i)[1];
            *z_iter = cloud.at(i)[2];
            ++x_iter;
            ++y_iter;
            ++z_iter;
        }
        rivz_puber_->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::spin(std::make_shared<HightMapNode>());
  rclcpp::shutdown();
  return 0;
}
