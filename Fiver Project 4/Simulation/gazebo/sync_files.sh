#!/usr/bin/env bash
# Copy the world + project to the Linux filesystem ($SIM_DIR), which is much faster than /mnt/c.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/sim_env.sh"
mkdir -p "$SIM_DIR/sitl"
if [ "$HERE" != "$SIM_DIR" ]; then
  rsync -a --delete "$HERE/models/" "$SIM_DIR/models/"
  rsync -a --delete "$HERE/worlds/" "$SIM_DIR/worlds/"
  cp "$HERE/survey.parm" "$HERE/sim_env.sh" "$HERE/area.tif" "$HERE/survey_gui.config" "$HERE/stop_sim.sh" "$SIM_DIR/"
  rsync -a --delete --exclude Simulation/gazebo/models --exclude __pycache__ "$HERE/../../" "$PROJECT_DIR/"
fi
