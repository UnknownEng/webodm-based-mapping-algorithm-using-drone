#!/usr/bin/env bash
# "Mission terminal" of the demo: fly the planned block.
source "$(dirname "${BASH_SOURCE[0]}")/demo_rc.sh"

say "STEP 2 - fly the mission (ROS 2 + MAVROS -> ArduPilot SITL in Gazebo)"
run python3 project/ROS2/mission.py plans/block_1.waypoints --yes
touch mission.done
exec bash
