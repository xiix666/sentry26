import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    namespace = LaunchConfiguration("namespace")
    relocate = LaunchConfiguration("relocate")
    mapping_mode = LaunchConfiguration("mapping_mode")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": map_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_arguments = ["--ros-args", "--log-level", log_level]

    static_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_lidar2lidar_1",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "-0.245", "--z", "-0.245",
                "--roll", "1.57", "--pitch", "-0.0", "--yaw", "0.0",
                "--frame-id", "lidar_link", "--child-frame-id", "lidar_link_1",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_map2odom",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_node_odom2lidarlink",
            output="screen",
            arguments=[
                "--x", "-0.0", "--y", "0.174", "--z", "0.358",
                "--roll", "-0.78", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "odom", "--child-frame-id", "odom_init",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_lidar2base",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.130", "--z", "-0.376",
                "--roll", "0.78", "--pitch", "-0.0", "--yaw", "0.0",
                "--frame-id", "lidar_link", "--child-frame-id", "base_link",
            ],
        ),
    ]

    localization_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_ros_driver2",
                namespace=namespace,
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="lidar_output",
                executable="lidar_output",
                name="lidar_output",
                namespace=namespace,
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                name="point_lio",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=common_arguments,
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                namespace=namespace,
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=common_arguments,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
                arguments=common_arguments,
            ),
        ],
    )

    relocation_nodes = GroupAction(
        condition=IfCondition(relocate),
        actions=[
            Node(
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=common_arguments,
            ),
        ],
    )

    mapping_nodes = GroupAction(
        condition=IfCondition(mapping_mode),
        actions=[
            Node(
                package="pc2scan",
                executable="pc2scan_node",
                name="pc2scan",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=common_arguments,
                remappings=[
                    ("cloud_in", "terrain_map_ext"),
                    ("scan", "slam_scan"),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=common_arguments,
                remappings=[
                    ("/map", "/slam_map"),
                    ("/map_metadata", "/slam_map_metadata"),
                    ("/map_updates", "/slam_map_updates"),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("relocate", default_value="False"),
            DeclareLaunchArgument("mapping_mode", default_value="True"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(bringup_dir, "map", "blank.yaml"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("autostart", default_value="True"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "nav_params.yaml"
                ),
            ),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            *static_transforms,
            localization_nodes,
            relocation_nodes,
            mapping_nodes,
        ]
    )
