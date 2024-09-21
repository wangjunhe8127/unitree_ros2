
#include "generate_boundary_points_node.hpp"

using std::placeholders::_1;
GenerateBoundaryPointsNode::GenerateBoundaryPointsNode()
    : Node("GenerateBoundaryPointsNode"){
  boundary_point_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/boundary_map", 10);
  run_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&GenerateBoundaryPointsNode::run_step, this));
  load_waypoints("/data/unitree_ros2/generate_boundary_points_node/boundary_points.txt");
}
void GenerateBoundaryPointsNode::run_step() {
    output_cloud_.header.stamp = this->get_clock()->now();
    output_cloud_.header.frame_id = "map"; // 设置适当的坐标系
    boundary_point_puber_->publish(output_cloud_);
}
bool GenerateBoundaryPointsNode::load_waypoints(const std::string &waypoint_path) {
  std::ifstream file(waypoint_path);
  if (!file.is_open()) {
    std::cerr << "无法打开文件：" << waypoint_path << std::endl;
    return false;
  }
  std::string line;
  double x, y, z;
  boundary_points_.clear();
  // 逐行读取数据
  while (getline(file, line)) {
    std::istringstream iss(line); // 将读取的行转换为字符串流
    std::string value;
    std::getline(iss, value, ',');
    x = std::stod(value);
    // 读取第二个数据
    std::getline(iss, value, ',');
    y = std::stod(value); // 转换为double
    // 读取第三个数据
    std::getline(iss, value);
    z = std::stod(value); // 转换为double
    // 输出读取的数据
    std::cout << "读取的数据: " << x << ", " << y << ", " << z << std::endl;
    boundary_points_.push_back(Eigen::Vector3d(x, y, z));
  }
  // 关闭文件
  file.close();

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < static_cast<int>(boundary_points_.size())-1; i++) {
    Eigen::Vector3d p_start = boundary_points_[i];
    Eigen::Vector3d p_end = boundary_points_[i + 1];
    
    // 计算两点之间的欧氏距离
    double distance = (p_end - p_start).norm();
    
    // 计算需要插值的点数，至少为 1
    int n_interp = static_cast<int>(std::floor(distance / unit_s_));
    if (n_interp < 1) n_interp = 1;
    
    // 计算插值步长的向量
    Eigen::Vector3d direction = (p_end - p_start).normalized();
    Eigen::Vector3d step = direction * (distance / n_interp);
    // 生成插值点
    for (int j = 0; j <= n_interp; ++j) {
        Eigen::Vector3d p_interp = p_start + step * j;
        pcl::PointXYZI point;
        point.x = p_interp.x();
        point.y = p_interp.y();
        point.z = p_interp.z();
        point.intensity = 0.0;
        cloud->points.push_back(point);
    }
  }
    cloud->width = cloud->points.size();
    cloud->height = 1; // 无序点云
    cloud->is_dense = true;

    
    pcl::toROSMsg(*cloud, output_cloud_);
  return true;
}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);                           // Initialize rclcpp
  rclcpp::spin(std::make_shared<GenerateBoundaryPointsNode>()); // Run ROS2 node
  rclcpp::shutdown();
  return 0;
}
