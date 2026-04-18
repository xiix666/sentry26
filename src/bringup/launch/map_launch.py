from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            # "0.33",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        # remappings=remappings,
    )

    map_path_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/chakfan/sentry/src/bringup/map/blank.yaml',
        description='Full path to map yaml file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},  # 自动启动/激活节点
            {'node_names': ['map_server']}  # 要激活的节点名
        ]
    )


    # 组装启动描述
    ld = LaunchDescription()
    ld.add_action(map_path_arg)
    ld.add_action(start_static_transform_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    return ld