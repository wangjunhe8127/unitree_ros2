#!/bin/bash
ros2 topic pub /control/dog_control_command msg/DogControlCommand "{
  control_mode: [0,0,0,0,0,0,0,0,1],
  point: {
    linear:{x:0.5,y:0.0,z:0.0},
    angular:{x:0.0,y:0.0,z:0.0}
  }
}"