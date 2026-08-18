import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos_best_effort = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)
def generate_launch_description():

    bringup_dir = get_package_share_directory("bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    lifecycle_nodes = ["map_server"]

    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value="/home/rps/sentry26/src/bringup/map/blank.yaml", description="Full path to map yaml file to load"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "nav_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    launch_dir = os.path.join(bringup_dir, "launch")
    start_map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        namespace=namespace,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
    )
    lidar_publish = Node(
        package="lidar_output",
        executable="lidar_output",
        name="lidar_output",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        namespace=namespace,
        parameters=[configured_params],
    )
    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,  
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )
    gicp_relocalization_node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pc2scan",
        executable="pc2scan_node",
        name="pc2scan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "slam_scan"),
        ],
    )
    start_sync_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "/slam_map"),
            ("/map_metadata", "/slam_map_metadata"),
            ("/map_updates", "/slam_map_updates"),
        ],
    )
    
    terrain_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )
    special_area = Node(
        package="special_area",
        executable="special_area_node",
        name="special_area",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "include", "robot_state_publisher_launch.py")
        ),
            launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "include", "perception_launch.py")
        ),
            launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    pointcloud_seg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "include", "pointcloud_seg_launch.py")
        ),
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(robot_state)

    ld.add_action(start_point_lio_node)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)

    ld.add_action(gicp_relocalization_node)
    ld.add_action(terrain_node)
    ld.add_action(special_area)

    return ld
