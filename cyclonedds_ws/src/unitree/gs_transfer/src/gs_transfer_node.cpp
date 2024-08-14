#include "gs_transfer_node.hpp"

#include <asio.hpp>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "state_datas.pb.h"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class GSTransferNode : public rclcpp::Node {
 public:
  GSTransferNode() : Node("gs_transfer_node") {
    // 声明参数
    this->declare_parameter<std::string>("udp_ip", "127.0.0.1");
    this->declare_parameter<int>("udp_port", 12345);

    // 获取参数值
    this->get_parameter("udp_ip", udp_ip_);
    this->get_parameter("udp_port", udp_port_);

    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        std::bind(&GSTransferNode::lowStateCallback, this,
                  std::placeholders::_1));
    sport_mode_state_sub_ =
        this->create_subscription<unitree_go::msg::SportModeState>(
            "/lf/sportmodestate", 10,
            std::bind(&GSTransferNode::sportModeStateCallback, this,
                      std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GSTransferNode::timerCallback, this));

    // 开启接收线程
    std::thread(&GSTransferNode::receiveDataOverUDP, this, udp_ip_, udp_port_)
        .detach();
  }

 private:
  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
    state_data_mapper_.mapLowStateToProtobuf(msg, state_data_);
    // RCLCPP_INFO(this->get_logger(), "LowState message converted to
    // protobuf.");
  }

  void sportModeStateCallback(
      const unitree_go::msg::SportModeState::SharedPtr msg) {
    state_data_mapper_.mapSportModeStateToProtobuf(msg, state_data_);
    // RCLCPP_INFO(this->get_logger(),
    // "SportModeState message converted to protobuf.");
  }

  void timerCallback() {
    std::string serialized_data;
    state_data_.SerializeToString(&serialized_data);
    std::cout << serialized_data.size() << std::endl;
    sendDataOverUDP(serialized_data, udp_ip_, udp_port_);
    // RCLCPP_INFO(this->get_logger(), "Protobuf data sent over UDP.");
  }

  void sendDataOverUDP(const std::string &data, const std::string &ip_address,
                       int port) {
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context);
    asio::ip::udp::endpoint endpoint(asio::ip::make_address(ip_address), port);
    socket.open(asio::ip::udp::v4());
    socket.send_to(asio::buffer(data), endpoint);
  }

  void receiveDataOverUDP(const std::string &ip_address, int port) {
    asio::io_context io_context;
    asio::ip::udp::socket socket(
        io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
    asio::ip::udp::endpoint sender_endpoint;

    while (rclcpp::ok()) {
      char recv_buf[65536];  // 大小根据需要调整，确保足够容纳数据
      std::size_t len =
          socket.receive_from(asio::buffer(recv_buf), sender_endpoint);

      quadruped::StateDatas received_state_data;
      if (received_state_data.ParseFromArray(recv_buf, len)) {
        RCLCPP_INFO(this->get_logger(),
                    "Received and deserialized Protobuf data successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to deserialize received Protobuf data.");
      }
    }
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_mode_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  StateDataMapper state_data_mapper_;
  quadruped::StateDatas state_data_;

  std::string udp_ip_;
  int udp_port_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GSTransferNode>());
  rclcpp::shutdown();
  return 0;
}
