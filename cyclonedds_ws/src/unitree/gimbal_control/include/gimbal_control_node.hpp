/**
 * @file gimbal_control_node.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <array>
#include <asio.hpp>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

class GimbalControl {
 public:
  explicit GimbalControl(const std::string& ip, int port)
      : endpoint_(asio::ip::address::from_string(ip), port),
        socket_(io_service_) {
    socket_.open(asio::ip::udp::v4());
    seq_ = 0;
  }

  void requestGimbalPose() {
    std::vector<uint8_t> data = serializGimbalPose(0x01, 0x04);
    std::vector<uint8_t> packet = generatePacket(0x25, data, true);
    sendPacket(packet);
  }

  std::vector<uint8_t> serializGimbalPose(uint8_t type, uint8_t freq) {
    std::vector<uint8_t> data(2, 0);
    data[0] = type;
    data[1] = freq;
    return data;
  }

  void controlGimbalAngle(double yaw, double pitch) {
    std::vector<uint8_t> data = serializeAngles(yaw, pitch);
    std::vector<uint8_t> packet = generatePacket(0x0E, data, true);
    sendPacket(packet);
  }

  std::vector<uint8_t> serializeAngles(double yaw, double pitch) {
    if (yaw > 270.0) {
      yaw = 270.0;
    } else if (yaw < -270.0) {
      yaw = -270.0;
    }
    if (pitch > 25.0) {
      pitch = 25.0;
    } else if (pitch < -90.0) {
      pitch = 90.0;
    }
    int16_t yaw_int = static_cast<int16_t>(yaw * 10);
    int16_t pitch_int = static_cast<int16_t>(pitch * 10);
    std::vector<uint8_t> data(4, 0);
    data[0] = yaw_int & 0xFF;
    data[1] = (yaw_int >> 8) & 0xFF;
    data[2] = pitch_int & 0xFF;
    data[3] = (pitch_int >> 8) & 0xFF;
    return data;
  }

  void requestAreaTemp(uint16_t start_x, uint16_t start_y, uint16_t end_x,
                       uint16_t end_y, uint8_t get_temp_flag) {
    std::vector<uint8_t> data =
        serializeAreaTemp(start_x, start_y, end_x, end_y, get_temp_flag);
    std::vector<uint8_t> packet = generatePacket(0x13, data, true);
    sendPacket(packet);
  }

  std::vector<uint8_t> serializeAreaTemp(uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y,
                                         uint8_t get_temp_flag) {
    std::vector<uint8_t> data(9, 0);
    data[0] = start_x & 0xFF;
    data[1] = (start_x >> 8) & 0xFF;
    data[2] = start_y & 0xFF;
    data[3] = (start_y >> 8) & 0xFF;
    data[4] = end_x & 0xFF;
    data[5] = (end_x >> 8) & 0xFF;
    data[6] = end_y & 0xFF;
    data[7] = (end_y >> 8) & 0xFF;
    data[8] = get_temp_flag & 0xFF;
    return data;
  }

  void receiveResponse() {
    std::array<uint8_t, 1024> recv_buf;
    asio::ip::udp::endpoint sender_endpoint;
    size_t len = socket_.receive_from(asio::buffer(recv_buf), sender_endpoint);

    if (len > 0) {
      parseResponse(
          std::vector<uint8_t>(recv_buf.begin(), recv_buf.begin() + len));
    }
  }

 private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint endpoint_;
  uint16_t seq_;

  std::vector<uint8_t> generatePacket(uint8_t cmd_id,
                                      const std::vector<uint8_t>& data,
                                      bool need_ack) {
    std::vector<uint8_t> packet;
    packet.push_back(0x55);  // STX low byte
    packet.push_back(0x66);  // STX high byte

    uint8_t ctrl = need_ack ? 0x01 : 0x00;
    packet.push_back(ctrl);

    uint16_t data_len = data.size();
    packet.push_back(data_len & 0xFF);         // Data_len low byte
    packet.push_back((data_len >> 8) & 0xFF);  // Data_len high byte

    packet.push_back(seq_ & 0xFF);         // SEQ low byte
    packet.push_back((seq_ >> 8) & 0xFF);  // SEQ high byte

    packet.push_back(cmd_id);  // CMD_ID

    packet.insert(packet.end(), data.begin(), data.end());  // DATA

    uint16_t crc = CRC16_cal(packet);
    packet.push_back(crc & 0xFF);         // CRC16 low byte
    packet.push_back((crc >> 8) & 0xFF);  // CRC16 high byte

    return packet;
  }

  void sendPacket(const std::vector<uint8_t>& packet) {
    socket_.send_to(asio::buffer(packet), endpoint_);
    seq_ = (seq_ + 1) % 65536;  // SEQ increment and reset at 65535
  }

  // CRC16校验计算
  uint16_t CRC16_cal(const std::vector<uint8_t>& data) const {
    uint16_t crc = 0;
    // int len = data.size();
    for (auto d : data) {
      uint8_t temp = (crc >> 8) & 0xFF;
      crc = (crc << 8) ^ crc16_tab[d ^ temp];
    }
    return crc;
  }

  // CRC16表
  static const uint16_t crc16_tab[256];

 public:
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  double max_temp{0.0};
  double min_temp{0.0};
  bool angle_updated{false};
  bool temp_updated{false};

  void parseResponse(const std::vector<uint8_t>& response) {
    if (response.size() < 10) return;  // Minimal packet size check

    // uint16_t seq = response[5] | (response[6] << 8);
    uint8_t cmd_id = response[7];

    // Handle response based on cmd_id
    if (cmd_id == 0x0D) {
      if (response.size() < 20) return;
      yaw = (response[8] | (response[9] << 8)) / 1.0;
      pitch = (response[10] | (response[11] << 8)) / 1.0;
      roll = (response[12] | (response[13] << 8)) / 1.0;
      angle_updated = true;
      std::cout << "Gimbal Pose - Yaw: " << yaw / 10.0
                << ", Pitch: " << pitch / 10.0 << ", Roll: " << roll / 10.0
                << std::endl;
    } else if (cmd_id == 0x13) {
      if (response.size() < 30) return;
      //   uint16_t startx = (response[8] | (response[9] << 8)) / 1.0;
      //   uint16_t starty = (response[10] | (response[11] << 8)) / 1.0;
      //   uint16_t endx = (response[12] | (response[13] << 8)) / 1.0;
      //   uint16_t endy = (response[14] | (response[15] << 8)) / 1.0;
      max_temp = ((response[16] | (response[17] << 8)) / 1.0) / 100.0;
      min_temp = ((response[18] | (response[19] << 8)) / 1.0) / 100.0;
      //   uint16_t temp_max_x = (response[20] | (response[21] << 8)) / 1.0;
      //   uint16_t temp_max_y = (response[22] | (response[23] << 8)) / 1.0;
      //   uint16_t temp_min_x = (response[24] | (response[25] << 8)) / 1.0;
      //   uint16_t temp_min_y = (response[26] | (response[27] << 8)) / 1.0;
      temp_updated = true;
    }
  }
};

// CRC16表初始化
const uint16_t GimbalControl::crc16_tab[256] = {
    0x0,    0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x420,  0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x630,  0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840,  0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50,  0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0xc60,  0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0xe70,  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0xa1,   0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x2b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x481,  0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x691,  0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1,  0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1,  0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,  0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0xed1,  0x1ef0};
