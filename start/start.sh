#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

source ~/.bashrc
source "${PROJECT_ROOT}/install/setup.bash"

ros2 launch bringup c_reloc_bringup_launch.py

if [ -n "$1" ]; then
    ros2 bag play "$1" --clock
fi

exit 0
