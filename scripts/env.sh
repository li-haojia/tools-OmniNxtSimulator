#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BASE_DIR="$(dirname "$SCRIPT_DIR")"

# Isaac Sim root directory
export ISAACSIM_PATH="/home/dji/service/isaac-sim/pkg/isaac-sim-2023.1.1"
# Isaac Sim python executable
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# ROS2
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$BASE_DIR/ros_bridge/dds_profile.xml

# ros2_bridge extension
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAACSIM_PATH/exts/omni.isaac.ros2_bridge/humble/lib