#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

ros2 run nav2_map_server map_saver_cli \
    -f "${PROJECT_ROOT}/src/bringup/map/map"
