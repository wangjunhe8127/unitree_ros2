#!/bin/bash
root_dir="/home/unitree/code/unitree_ros2"
SESSION_NAME="multi_pane_session"
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Closing existing tmux session: $SESSION_NAME"
    tmux kill-session -t $SESSION_NAME
fi
tmux new-session -d -s $SESSION_NAME -n "main_window"

# loc waypoint
tmux send-keys "source /opt/ros/foxy/setup.sh" C-m
tmux send-keys "source $root_dir/setup.sh" C-m
tmux send-keys "source $root_dir/install/setup.sh" C-m
tmux send-keys "ros2 run routing_test_node routing_test_node_exe" C-m
# map
tmux split-window -h
tmux send-keys "source /opt/ros/foxy/setup.sh" C-m
tmux send-keys "source $root_dir/setup.sh" C-m
tmux send-keys "source $root_dir/install/setup.sh" C-m
tmux send-keys "ros2 launch terrain_analysis terrain_analysis.launch" C-m
# map
tmux split-window -h
tmux send-keys "source /opt/ros/foxy/setup.sh" C-m
tmux send-keys "source $root_dir/setup.sh" C-m
tmux send-keys "source $root_dir/install/setup.sh" C-m
tmux send-keys "ros2 run hight_map_node hight_map_node_exe " C-m
# plan
tmux split-window -v
tmux send-keys "source /opt/ros/foxy/setup.sh" C-m
tmux send-keys "source $root_dir/setup.sh" C-m
tmux send-keys "source $root_dir/install/setup.sh" C-m
tmux send-keys "ros2 launch local_planner local_planner.launch" C-m

# control 需手动回车运行
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "source /opt/ros/foxy/setup.sh" C-m
tmux send-keys "source $root_dir/setup.sh" C-m
tmux send-keys "source $root_dir/install/setup.sh" C-m
tmux send-keys "ros2 launch local_planner controller.launch"

tmux select-layout tiled
tmux attach-session -t $SESSION_NAME
