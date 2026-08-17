#!/bin/bash

# 检查是否传入了目录名参数
if [ $# -eq 0 ]; then
    echo "错误：请提供一个目录名作为参数！"
    echo "使用方法：$0 <目录名>"
    echo "示例：$0 lidar_test_20260320"
    exit 1
fi

# 获取命令行传入的目录名
SUB_DIR=$1

# 定义完整的 bag 保存路径
BAG_BASE_PATH="/home/rps/sentry26/bag"
BAG_FULL_PATH="${BAG_BASE_PATH}/${SUB_DIR}"

# 创建目录（-p 参数确保父目录存在，且已存在时不报错）
# mkdir -p "${BAG_FULL_PATH}"

# 检查目录创建是否成功
# if [ ! -d "${BAG_FULL_PATH}" ]; then
#     echo "错误：无法创建目录 ${BAG_FULL_PATH}"
#     exit 1
# fi

# 定义 bag 文件的输出名称（使用目录名作为基础名）
BAG_OUTPUT_NAME="${BAG_FULL_PATH}"

# 开始录制 ros2 bag
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