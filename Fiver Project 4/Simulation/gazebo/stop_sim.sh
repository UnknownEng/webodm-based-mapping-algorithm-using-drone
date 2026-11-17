#!/usr/bin/env bash
# Stop every simulation process (closing tmux alone leaves gz/arducopter running).
# Patterns are anchored to the start of the command line so they never match
# a tmux server or shell whose command line merely *mentions* these programs.
tmux kill-session -t survey 2>/dev/null
PATS=("^gz sim" "^ruby .*gz sim" "/arducopter --model JSON" "^/opt/ros/[a-z]*/lib/mavros/mavros_node"
      "^/opt/ros/[a-z]*/lib/ros_gz_bridge/parameter_bridge" "^python3 .*ROS2/images.py"
      "^python3 .*ROS2/mission.py" "^python3 .*ROS2/check_status.py" "^python3 .*demo_viewer.py")
for p in "${PATS[@]}"; do pkill -INT -f "$p" 2>/dev/null; done
sleep 2
for p in "${PATS[@]}"; do pkill -KILL -f "$p" 2>/dev/null; done
true
