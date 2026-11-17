#!/usr/bin/env bash
# Environment for the survey simulation (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, ArduPilot SITL).
# Override any path by exporting it before sourcing this file.
source /opt/ros/jazzy/setup.bash

export SIM_DIR="${SIM_DIR:-$HOME/survey_sim}"                         # runtime copy of this folder
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/sim-deps/ardupilot}"      # built with: ./waf configure --board sitl && ./waf copter
export ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/fyp_deps/ardupilot_gazebo}"
export PROJECT_DIR="${PROJECT_DIR:-$SIM_DIR/project}"                  # copy of "Fiver Project 4"

# WSL2: render on the GPU through Mesa's D3D12 driver instead of llvmpipe (CPU).
# Pick the GPU by (part of) its name; ignored on normal Linux machines.
if grep -qi microsoft /proc/version 2>/dev/null; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
fi

export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$SIM_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$SIM_DIR/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"

# Take-off point (must match gen_world.py --home-lat/--home-lon/--home-alt)
export HOME_LOC="${HOME_LOC:--35.363262,149.165237,584,0}"
