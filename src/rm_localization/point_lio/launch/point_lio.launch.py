import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )
    static_tf_livox_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="livox_to_lidar_link_tf",
        output="screen",
        arguments=[

            "0.0", "0.0", "0.0",

            "0.0", "0.0", "0.0",

            "lidar_link", "livox_frame"
        ],
        parameters=[{"use_sim_time": False}],
    )
    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(static_tf_livox_to_lidar)
    ld.add_action(start_point_lio_node)

    return ld
