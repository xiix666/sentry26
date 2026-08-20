#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

export ROS_LOG_DIR="${PROJECT_ROOT}/log"

source ~/.bashrc
source "${PROJECT_ROOT}/install/setup.bash"

cd "${PROJECT_ROOT}"

ros2 launch bringup nav_launch.py

exit 0
