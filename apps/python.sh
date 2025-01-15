#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR

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

function check_exts_dir {
    EXTS_DIR=$SCRIPT_DIR/../source/exts
    if [ -d "$EXTS_DIR" ]; then
        echo "Pass: $EXTS_DIR exists"
    else
        ln -s /isaac-sim/exts $SCRIPT_DIR/../source/exts
    fi
}

if [ $# -lt 1 ]; then
    usage
fi

TARGET_SCRIPT=$1
check_exts_dir
check_target_script $TARGET_SCRIPT
$SCRIPT_DIR/../isaaclab.sh -p $TARGET_SCRIPT "$@" --headless