from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
import os
from ament_index_python.packages import get_package_share_directory
bringup_dir = get_package_share_directory("bringup")
launch_dir = os.path.join(bringup_dir, "launch")
from launch.launch_description_sources import PythonLaunchDescriptionSource
bringup_dir = get_package_share_directory("bringup")
launch_dir = os.path.join(bringup_dir, "launch")

port_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, "include", "serial_driver_launch.py")),
)
def generate_launch_description():
    return LaunchDescription([   
        Node(
            package='omni_perception',
            executable='omni_perception',  
            name='omni_perception',
            output='screen',
            parameters=[
                {'red_blue': True},
                {'transform_red_x': 3.43},
                {'transform_red_y': 8.04},
                {'transform_blue_x': 23.922},
                {'transform_blue_y': 8.15},
                {'aim_odom_frame': "odom_aim"},
                {'gimbal_frame': "big_gimbal_link"}
            ]
        )
        # port_cmd
    ])