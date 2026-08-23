import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():

    bringup_dir = get_package_share_directory("bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    try:
        rm_decision_dir = get_package_share_directory("rm2026_decision")
        decisionparams_file = os.path.join(
            rm_decision_dir, "config", "rm2026_decision_params.yaml"
        )
        decision_available = True
    except PackageNotFoundError:
        decisionparams_file = ""
        decision_available = False

    namespace = LaunchConfiguration("namespace")
    relocate = LaunchConfiguration("relocate")
    mapping_mode = LaunchConfiguration("mapping_mode")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_omni_perception = LaunchConfiguration("use_omni_perception")
    use_map_save = LaunchConfiguration("use_map_save")
    use_decision = LaunchConfiguration("use_decision")
    map_save_path = LaunchConfiguration("map_save_path")

    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

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

    declare_relocate_cmd = DeclareLaunchArgument(
        "relocate", default_value="False", description="Whether to relocate"
    )
    declare_mapping_mode_cmd = DeclareLaunchArgument(
        "mapping_mode", default_value="True", description="Whether to build a map"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(

        "map",
        default_value=os.path.join(bringup_dir, "map", "map_uc.yaml"),
        description="Full path to map yaml file to load",

    )                                                                                                 

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "nav_params_1.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RVIZ"
    )
    declare_use_omni_perception_cmd = DeclareLaunchArgument(
        "use_omni_perception",
        default_value="False",
        description="Start the optional four-camera omni perception adapter",
    )
    declare_use_map_save_cmd = DeclareLaunchArgument(
        "use_map_save", default_value="False"
    )
    declare_use_decision_cmd = DeclareLaunchArgument(
        "use_decision",
        default_value="True" if decision_available else "False",
        description="Start rm2026_decision when the private package is installed",
    )
    declare_map_save_path_cmd = DeclareLaunchArgument(
        "map_save_path",
        default_value=os.path.join(os.path.expanduser("~"), "sentry26_maps", "map"),
        description="Output prefix used by the periodic map saver",
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "include", "perception_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "include", "localization_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "relocate": relocate,
                    "mapping_mode": mapping_mode,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "include", "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    port_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "include", "serial_driver_launch.py")),
    )
    omni_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "include", "omni_perception_launch.py")),
    )
    rm26_decision = Node(
        condition=IfCondition(use_decision),
        package="rm2026_decision",
        executable="rm2026_decision_node",
        name="rm2026_decision_node",
        output="screen",
        parameters=[decisionparams_file] if decision_available else [],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "include", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    save_map_cmd = Node(
        condition=IfCondition(use_map_save),
        package="map_save",
        executable="map_save",
        name="map_save",
        output="screen",
        parameters=[configured_params,
        {
            "save_interval": 20.0,
            "save_path": map_save_path,
            "map_topic": "/slam_map",
        }
        ],
    )
    delayed_omni_node = TimerAction(
        condition=IfCondition(use_omni_perception),
        period=10.0,
        actions=[omni_cmd]
    )

    delayed_decision_node = TimerAction(
        condition=IfCondition(use_decision),
        period=10.0,
        actions=[rm26_decision],
    )
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_relocate_cmd)
    ld.add_action(declare_mapping_mode_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_omni_perception_cmd)
    ld.add_action(declare_use_map_save_cmd)
    ld.add_action(declare_use_decision_cmd)
    ld.add_action(declare_map_save_path_cmd)

    ld.add_action(bringup_cmd_group)
    ld.add_action(delayed_omni_node)
    ld.add_action(port_cmd)
    ld.add_action(rviz_cmd)

    ld.add_action(save_map_cmd)

    ld.add_action(delayed_decision_node)

    return ld
