#include "gs_transfer_node.hpp"

StateDataMapper::StateDataMapper() {}

void StateDataMapper::mapLowStateToProtobuf(
    const unitree_go::msg::LowState::SharedPtr& low_state_msg,
    quadruped::StateDatas& state_data) {
  // 清空重复字段数据
  state_data.mutable_body_location()->clear_foot_force();
  auto imu_state_proto = state_data.mutable_imu_state();
  imu_state_proto->clear_quaternion();
  imu_state_proto->clear_gyroscope();
  imu_state_proto->clear_accelerometer();
  imu_state_proto->clear_rpy();

  // 映射 foot_force
  for (int i = 0; i < 4; ++i) {
    state_data.mutable_body_location()->add_foot_force(
        low_state_msg->foot_force[i]);
  }

  // 映射 IMUState
  for (int i = 0; i < 4; ++i) {
    imu_state_proto->add_quaternion(low_state_msg->imu_state.quaternion[i]);
  }
  for (int i = 0; i < 3; ++i) {
    imu_state_proto->add_gyroscope(low_state_msg->imu_state.gyroscope[i]);
    imu_state_proto->add_accelerometer(
        low_state_msg->imu_state.accelerometer[i]);
    imu_state_proto->add_rpy(low_state_msg->imu_state.rpy[i]);
  }
  imu_state_proto->set_temperature(
      low_state_msg->temperature_ntc1);  // 假设使用第一个温度传感器

  // 映射 BMS 数据
  auto battery_state_proto = state_data.mutable_battery_state();
  battery_state_proto->set_status(low_state_msg->bms_state.status);
  battery_state_proto->set_soc(low_state_msg->bms_state.soc);
  battery_state_proto->set_current(low_state_msg->bms_state.current);
  battery_state_proto->set_cycle(low_state_msg->bms_state.cycle);
}

void StateDataMapper::mapSportModeStateToProtobuf(
    const unitree_go::msg::SportModeState::SharedPtr& sport_mode_state_msg,
    quadruped::StateDatas& state_data) {
  // 清空重复字段数据
  auto global_location_proto = state_data.mutable_global_location();
  global_location_proto->clear_position();
  global_location_proto->clear_velocity();
  state_data.mutable_body_location()->clear_foot_force();
  state_data.mutable_body_location()->clear_foot_position_body();
  state_data.mutable_body_location()->clear_foot_speed_body();

  state_data.set_obstacle_avoid_mode(false);  // 示例赋值，可以根据需要调整
  state_data.set_sport_mode("normal");  // 示例赋值，可以根据需要调整
  state_data.set_high_level_mode(sport_mode_state_msg->mode);
  state_data.set_foot_raise_height(sport_mode_state_msg->foot_raise_height);
  state_data.set_gait_type(sport_mode_state_msg->gait_type);

  // 映射 GlobalLocation
  for (int i = 0; i < 3; ++i) {
    global_location_proto->add_position(sport_mode_state_msg->position[i]);
    global_location_proto->add_velocity(sport_mode_state_msg->velocity[i]);
  }
  global_location_proto->set_yaw_speed(sport_mode_state_msg->yaw_speed);

  // 映射 foot_force
  for (int i = 0; i < 4; ++i) {
    state_data.mutable_body_location()->add_foot_force(
        sport_mode_state_msg->foot_force[i]);
  }

  // 映射 foot_position_body 和 foot_speed_body
  for (int i = 0; i < 12; ++i) {
    state_data.mutable_body_location()->add_foot_position_body(
        sport_mode_state_msg->foot_position_body[i]);
    state_data.mutable_body_location()->add_foot_speed_body(
        sport_mode_state_msg->foot_speed_body[i]);
  }

  // 设置 body_height
  state_data.mutable_body_location()->set_body_height(sport_mode_state_msg->body_height);
}
