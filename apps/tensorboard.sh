#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR

function usage {
    echo "Usage: $0 <log_dir>"
    exit 1
}

function check_exts_dir {
    EXTS_DIR=$SCRIPT_DIR/../source/exts
    if [ -d "$EXTS_DIR" ]; then
        echo "Pass: $EXTS_DIR exists"
    else
        ln -s /isaac-sim/exts $SCRIPT_DIR/../source/exts
    fi
}

if [ $# -ne 1 ]; then
    usage
fi

TARGET_DIR=$1
check_exts_dir
$SCRIPT_DIR/../isaaclab.sh -p -m tensorboard.main --logdir $TARGET_DIR