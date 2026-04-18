#!/bin/bash

export ROS_LOG_DIR=/home/rps/sentry26/log
# export ROS_DOMAIN_ID=2
source ~/.bashrc

source /home/rps/sentry26/install/setup.bash

cd /home/rps/sentry26

ros2 launch bringup nav_launch.py

exit 0
