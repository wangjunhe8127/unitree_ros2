#include "gimbal_control_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "unitree_go/msg/gimbal_control.hpp"
#include "unitree_go/msg/gimbal_status.hpp"

class RTSPStreamer : public rclcpp::Node {
 public:
  RTSPStreamer() : Node("rtsp_streamer") {
    // Declare and get parameters
    this->declare_parameter<std::string>("rtsp_url_master",
                                         "rtsp://127.0.0.1:8554/test");
    this->declare_parameter<std::string>("rtsp_url_slave",
                                         "rtsp://127.0.0.1:8555/test2");
    this->declare_parameter<std::string>("topic_name_master", "vis_image");
    this->declare_parameter<std::string>("topic_name_slave", "ir_image");
    this->declare_parameter<std::string>("udp_ip", "127.0.0.1");
    this->declare_parameter<int>("port", 10025);
    this->declare_parameter<bool>("master_enable", true);
    this->declare_parameter<bool>("slave_enable", false);

    rtsp_url_master =
        this->get_parameter("rtsp_url_master").as_string();
    rtsp_url_slave =
        this->get_parameter("rtsp_url_slave").as_string();
    std::string topic_name_master =
        this->get_parameter("topic_name_master").as_string();
    std::string topic_name_slave =
        this->get_parameter("topic_name_slave").as_string();
    std::string udp_ip = this->get_parameter("udp_ip").as_string();
    int port = this->get_parameter("port").as_int();
    bool master_enable = this->get_parameter("master_enable").as_bool();
    bool slave_enable = this->get_parameter("slave_enable").as_bool();

    gimbal_control_ = std::make_shared<GimbalControl>(udp_ip, port);

    gimbal_status_publisher_ =
        this->create_publisher<unitree_go::msg::GimbalStatus>("gimbal_status",
                                                              10);
    gimbal_control_subscriber_ =
        this->create_subscription<unitree_go::msg::GimbalControl>(
            "gimbal_control", 10,
            std::bind(&RTSPStreamer::gimbal_msg_callback, this,
                      std::placeholders::_1));

    if (master_enable) {
      // Open the first RTSP stream
      cap_1_.open(rtsp_url_master);
      if (!cap_1_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream 1: %s",
                     rtsp_url_master.c_str());
        rclcpp::shutdown();
      }
      publisher_1_ = this->create_publisher<sensor_msgs::msg::Image>(
          topic_name_master, 10);
      // Timers
      timer_1_ = this->create_wall_timer(
          std::chrono::milliseconds(10),  // 20 Hz
          std::bind(&RTSPStreamer::stream_callback_1, this));
    }

    if (slave_enable) {
      // Open the second RTSP stream
      cap_2_.open(rtsp_url_slave);
      if (!cap_2_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream 2: %s",
                     rtsp_url_slave.c_str());
        rclcpp::shutdown();
      }
      // Publishers
      publisher_2_ =
          this->create_publisher<sensor_msgs::msg::Image>(topic_name_slave, 10);
      timer_2_ = this->create_wall_timer(
          std::chrono::milliseconds(100),  // 20 Hz
          std::bind(&RTSPStreamer::stream_callback_2, this));
    }

    timer_3_ = this->create_wall_timer(
        std::chrono::milliseconds(30),  // 20 Hz
        std::bind(&RTSPStreamer::gimbal_control_callback, this));
  }

 private:
  void gimbal_msg_callback(
      const unitree_go::msg::GimbalControl::SharedPtr msg) {
    target_yaw_ = msg->yaw;
    target_pitch_ = msg->pitch;
    target_updated_ = true;
  }

  void gimbal_control_callback() {
    // gimbal_control_->angle_updated = true;
    // gimbal_control_->temp_updated = true;
    
    // update angle and temp
    if (!gimbal_control_->temp_updated) {
      //gimbal_control_->requestGimbalPose();
      gimbal_control_->requestAreaTemp(0, 0, 20, 20, 2);
    }

    if (gimbal_control_->angle_updated && target_updated_) {
      target_updated_ = false;
      if (std::abs(target_pitch_ - gimbal_control_->pitch) > 2.0 ||
          std::abs(target_yaw_ - gimbal_control_->yaw) > 2.0) {
        std::cout << "target pitch:" << target_pitch_ << " target_yaw:" << target_yaw_ << std::endl;
        gimbal_control_->controlGimbalAngle(target_yaw_, target_pitch_);
      }
    }else{
      gimbal_control_->requestGimbalPose();
    }

    gimbal_control_->receiveResponse();

    // update msg
    auto gimbal_status_msg = std::make_shared<unitree_go::msg::GimbalStatus>();

    // update msg
    gimbal_status_msg->yaw = gimbal_control_->yaw;
    gimbal_status_msg->pitch = gimbal_control_->pitch;
    gimbal_status_msg->roll = gimbal_control_->roll;
    gimbal_status_msg->max_temp = gimbal_control_->max_temp;
    gimbal_status_msg->min_temp = gimbal_control_->min_temp;
    // 设置时间戳
    gimbal_status_msg->angle_timestamp.sec = 0;
    gimbal_status_msg->angle_timestamp.nanosec = 0;
    gimbal_status_msg->temp_timestamp.sec = 0;
    gimbal_status_msg->temp_timestamp.nanosec = 0;
    gimbal_status_publisher_->publish(*gimbal_status_msg);
  }

  void stream_callback_1() {
    cv::Mat frame;
    if (cap_1_.read(frame)) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                     .toImageMsg();
      publisher_1_->publish(*msg);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to capture frame from RTSP stream 1");
      cap_1_.open(rtsp_url_master);
      if (!cap_1_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream 1: %s",
                     rtsp_url_master.c_str());
        rclcpp::shutdown();
      }
    }
  }

  void stream_callback_2() {
    cv::Mat frame;
    if (cap_2_.read(frame)) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                     .toImageMsg();
      publisher_2_->publish(*msg);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to capture frame from RTSP stream 2");
      cap_2_.open(rtsp_url_slave);
      if (!cap_2_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream 1: %s",
                     rtsp_url_master.c_str());
        rclcpp::shutdown();
      }
    }
  }

  cv::VideoCapture cap_1_, cap_2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_1_,
      publisher_2_;
  rclcpp::TimerBase::SharedPtr timer_1_, timer_2_, timer_3_;
  std::shared_ptr<GimbalControl> gimbal_control_;
  rclcpp::Publisher<unitree_go::msg::GimbalStatus>::SharedPtr
      gimbal_status_publisher_;
  rclcpp::Subscription<unitree_go::msg::GimbalControl>::SharedPtr
      gimbal_control_subscriber_;

  // control msg
  double target_yaw_{0.0};
  double target_pitch_{0.0};
  bool target_updated_{false};
  std::string rtsp_url_master;
  std::string rtsp_url_slave;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RTSPStreamer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
