#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BASE_DIR="$(dirname "$SCRIPT_DIR")"

cp $BASE_DIR/scripts/docker-compose.yaml $BASE_DIR/IsaacLab/docker/docker-compose.yaml
python3 $BASE_DIR/IsaacLab/docker/container.py start ros2