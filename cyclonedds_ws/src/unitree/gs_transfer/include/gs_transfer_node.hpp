/**
 * @file gs_transfer_node.hpp
 * @brief
 * @version 0.1
 * @date 2024-08-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "state_datas.pb.h"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class StateDataMapper {
 public:
  StateDataMapper();
  void mapLowStateToProtobuf(
      const unitree_go::msg::LowState::SharedPtr& low_state_msg,
      quadruped::StateDatas& state_data);  // NOLINT
  void mapSportModeStateToProtobuf(
      const unitree_go::msg::SportModeState::SharedPtr& sport_mode_state_msg,
      quadruped::StateDatas& state_data);  // NOLINT
};
