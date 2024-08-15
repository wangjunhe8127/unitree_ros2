#!/bin/bash
ros2 bag record \
    /control/dog_control_command \
    /control/dog_report_common \
    /lf/lowstate \
    /lf/sportmodestate \
    /lowcmd \
    /lowstate \
    /sportmodestate \
    /uslam/cloud_map \
    /uslam/localization/cloud_world \
    /uslam/localization/odom \
    /uslam/navigation/global_path \
    /utlidar/cloud \
    /utlidar/cloud_deskewed \
    /utlidar/grid_map \
    /utlidar/height_map \
    /utlidar/height_map_array \
    /utlidar/imu \
    /utlidar/lidar_state \
    /utlidar/range_info \
    /utlidar/range_map \
    /utlidar/robot_odom \
    /utlidar/robot_pose \
