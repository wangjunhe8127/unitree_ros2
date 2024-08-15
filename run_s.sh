#!/bin/bash
ros2 topic pub /control/dog_control_command unitree_go/msg/DogControlCommand "{
  control_mode: [0,0,0,0,0,0,1,0,0]
}"
