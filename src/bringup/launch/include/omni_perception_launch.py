from launch import LaunchDescription
from launch_ros.actions import Node


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

    ])
