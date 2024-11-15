#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

function usage {
    echo "Usage: $0 <target_script>"
    exit 1
}

function check_target_script {
    if [ ! -f $1 ]; then
        echo "Error: $1 not found"
        usage
    fi
}

if [ $# -ne 1 ]; then
    usage
fi

TARGET_SCRIPT=$1
check_target_script $TARGET_SCRIPT
$SCRIPT_DIR/../isaaclab.sh -p $TARGET_SCRIPT --headless --livestream 1
# --kit_args "--enable omni.isaac.ros2_bridge"