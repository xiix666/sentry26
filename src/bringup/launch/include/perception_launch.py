import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "nav_params.yaml"
                ),
            ),
            Node(
                package="pointcloud_frame_trans",
                executable="pointcloud_frame_trans",
                name="pointcloud_frame_trans",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="terrain_analysis",
                executable="terrainAnalysisExt",
                name="terrain_analysis_ext",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="special_area",
                executable="special_area_node",
                name="special_area",
                output="screen",
                parameters=[configured_params],
            ),
        ]
    )
