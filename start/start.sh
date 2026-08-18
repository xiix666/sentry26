#!/bin/bash

source ~/.bashrc

source /home/xx/sentry26/install/setup.bash

ros2 launch bringup c_reloc_bringup_launch.py 

ros2 bag play /home/xx/sentry26/bag/0802/0802_1_0.db3 --clock

exit 0
