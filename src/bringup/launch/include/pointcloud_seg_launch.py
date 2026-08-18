from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    bringup_dir = get_package_share_directory("bringup")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", 
        default_value="", 
        description="Top-level namespace"
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "nav_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    param_substitutions = {}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(Node(
        package='pointcloud_frame_trans',
        executable='pointcloud_frame_trans',
        name='pointcloud_frame_trans',
        output='screen',
        parameters=[
            configured_params,
        ]
    ))

    return ld
