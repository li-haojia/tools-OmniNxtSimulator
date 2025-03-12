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
SCRIPT_ARGS=${@:2}
check_exts_dir
check_target_script $TARGET_SCRIPT
$SCRIPT_DIR/../isaaclab.sh -p $TARGET_SCRIPT $SCRIPT_ARGS --kit_args "--enable isaacsim.ros2.bridge --enable isaacsim.asset.gen.omap"

# Tracy profiler
# --enable omni.kit.profiler.tracy --/profiler/enabled=true --/app/profilerBackend=tracy --/privacy/externalBuild=0 --/app/profileFromStart=true --/profiler/gpu=true --/profiler/gpu/tracyInject/enabled=true --/profiler/gpu/tracyInject/msBetweenClockCalibration=0 --/app/profilerMask=1 --/profiler/channels/carb.tasking/enabled=false --/profiler/channels/carb.events/enabled=false --/plugins/carb.profiler-tracy.plugin/fibersAsThreads=false"