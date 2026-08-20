#!/bin/bash

if [ $# -eq 0 ]; then
    echo "错误：请提供一个目录名作为参数！"
    echo "使用方法：$0 <目录名>"
    echo "示例：$0 lidar_test_20260320"
    exit 1
fi

SUB_DIR=$1
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

BAG_BASE_PATH="${PROJECT_ROOT}/bag"
BAG_FULL_PATH="${BAG_BASE_PATH}/${SUB_DIR}"

BAG_OUTPUT_NAME="${BAG_FULL_PATH}"

echo "开始录制 bag 文件，保存路径：${BAG_OUTPUT_NAME}"

cleanup()
{
    echo ""
    echo "正在停止 ros2 bag..."
    kill -SIGINT "$ROS2_BAG_PID"
    wait "$ROS2_BAG_PID"

    echo "Bag 已安全保存"
    exit 0
}

trap cleanup SIGINT SIGTERM

ros2 bag record \
    -o "${BAG_OUTPUT_NAME}" \
    /livox/lidar \
    /livox/imu &

ROS2_BAG_PID=$!

wait "$ROS2_BAG_PID"
