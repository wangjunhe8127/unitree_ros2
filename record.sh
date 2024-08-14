#!/bin/bash
ros2 bag record \
    /control/dog_report_common \
    /control/dog_control_command \
    lf/lowstate \
    lf/sportmodestate