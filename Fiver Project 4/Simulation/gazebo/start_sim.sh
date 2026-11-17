#!/usr/bin/env bash
# Start the survey simulation in a tmux session "survey":
#   window gz     - Gazebo Harmonic (server; GUI too unless HEADLESS=1)
#   window sitl   - ArduCopter SITL (JSON backend, talks to the Gazebo ArduPilot plugin)
#   window mavros - MAVROS 2 (tcp://127.0.0.1:5760)
#   window bridge - ros_gz_bridge: Gazebo camera -> ROS 2 /survey_camera/image
# Stop everything with:  tmux kill-session -t survey
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/sim_env.sh"

# Run from the Linux filesystem (much faster than /mnt/c)
[ "${NO_SYNC:-0}" = "1" ] || bash "$HERE/sync_files.sh"

GZ_FLAGS="-r -v 3"
if [ "${HEADLESS:-0}" = "1" ]; then
  GZ_FLAGS="$GZ_FLAGS -s"
else
  GZ_FLAGS="$GZ_FLAGS --gui-config $SIM_DIR/survey_gui.config"
fi

bash "$HERE/stop_sim.sh"
tmux new-session -d -s survey -n gz "source $SIM_DIR/sim_env.sh; gz sim $GZ_FLAGS $SIM_DIR/worlds/survey.sdf 2>&1 | tee $SIM_DIR/gz.log"
sleep 8
tmux new-window -t survey -n sitl "source $SIM_DIR/sim_env.sh; cd $SIM_DIR/sitl && \
  $ARDUPILOT_DIR/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home $HOME_LOC \
  --defaults $ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$SIM_DIR/survey.parm \
  2>&1 | tee $SIM_DIR/sitl.log"
sleep 3
tmux new-window -t survey -n mavros "source $SIM_DIR/sim_env.sh; ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 2>&1 | tee $SIM_DIR/mavros.log"
tmux new-window -t survey -n bridge "source $SIM_DIR/sim_env.sh; ros2 run ros_gz_bridge parameter_bridge /survey_camera/image@sensor_msgs/msg/Image[gz.msgs.Image 2>&1 | tee $SIM_DIR/bridge.log"
echo "Simulation started (tmux session 'survey'). Attach with: tmux attach -t survey"
