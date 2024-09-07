#include "hight_map_node.hpp"

using std::placeholders::_1;

HightMapNode::HightMapNode() : Node("HightMapNode") {
  sport_loc_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10, std::bind(&HightMapNode::sport_loc_callback, this, std::placeholders::_1));
  world_loc_suber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10, std::bind(&HightMapNode::world_loc_callback, this, std::placeholders::_1));
  hight_map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
      "/utlidar/height_map_array", 10, std::bind(&HightMapNode::hight_map_callback, this, std::placeholders::_1));
  world_hight_map_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/world_hight_map", 10);
  rivz_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/local_hight_map", 10);
  local_hight_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}
void HightMapNode::sport_loc_callback(const unitree_go::msg::SportModeState::ConstSharedPtr data) {
  sport_loc_.pose.pose.position.x = data->position[0];
  sport_loc_.pose.pose.position.y = data->position[1];
  sport_loc_.pose.pose.position.z = data->position[2];
  sport_loc_.pose.pose.orientation.x = data->imu_state.quaternion[1];
  sport_loc_.pose.pose.orientation.y = data->imu_state.quaternion[2];
  sport_loc_.pose.pose.orientation.z = data->imu_state.quaternion[3];
  sport_loc_.pose.pose.orientation.w = data->imu_state.quaternion[0];
  tf2::fromMsg(sport_loc_.pose.pose, transform_local_);
}
void HightMapNode::world_loc_callback(const nav_msgs::msg::Odometry::ConstSharedPtr data) {
  world_loc_ = *data;
  tf2::fromMsg(world_loc_.pose.pose, transform_world_);
}
void HightMapNode::hight_map_callback(const unitree_go::msg::HeightMap::ConstSharedPtr msg) {
  local_hight_map_->clear();
  int width = msg->width;
  int height = msg->height;
  float resolution = msg->resolution;
  float originX = msg->origin[0];
  float originY = msg->origin[1];
  std::vector<std::array<float, 3>> cloud;
  int index;
  // std::cout<< "width: " << width <<std::endl;
  // std::cout<< "height: " << height <<std::endl;
  // std::cout<< "originX: " << originX <<std::endl;
  // std::cout<< "originY: " << originY <<std::endl;
  // std::cout<< "resolution: " << resolution <<std::endl;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      index = ix + width * iy;
      if (msg->data[index] == 1.0e9) { // skip empty cell value which is set to 1.0e9
        continue;
      }
      pcl::PointXYZI point;
      point.x = ix * resolution + originX;
      point.y = iy * resolution + originY;
      point.z = msg->data[index];
      point.intensity = msg->data[index];
      local_hight_map_->points.push_back(point);
      // std::cout<<"point: " << pt[0] << " " << pt[1] << " " << pt[2] <<std::endl;
    }
  }
  local_hight_map_->width = local_hight_map_->points.size();
  local_hight_map_->height = 1;
  local_hight_map_->is_dense = true;
  convert_cloudpoints();
  pubrviz();
}
void HightMapNode::convert_cloudpoints() {
    tf2::Transform transform_local_to_world = transform_world_ * transform_local_.inverse();
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform_local_to_world, eigen_transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr world_hight_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*local_hight_map_, *world_hight_map, eigen_transform);

    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*world_hight_map, output_cloud);
    output_cloud.header.stamp = this->now();
    output_cloud.header.frame_id = "map";
    world_hight_map_puber_->publish(output_cloud);
}
void HightMapNode::pubrviz() {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.height = 1;
        msg.width = static_cast<int>(local_hight_map_->points.size());
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
        for (int i = 0; i < static_cast<int>(local_hight_map_->points.size()); ++i) {
            *x_iter = local_hight_map_->points.at(i).x;
            *y_iter = local_hight_map_->points.at(i).y;
            *z_iter = local_hight_map_->points.at(i).z;
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
