from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # -------------------------- 1. 获取包路径 --------------------------
    bringup_dir = get_package_share_directory("bringup")
    
    # -------------------------- 2. 声明Launch配置变量 --------------------------
    # 必须在generate_launch_description函数内声明，且先声明后使用
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
    
    # -------------------------- 3. 获取Launch配置变量 --------------------------
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    
    # -------------------------- 4. 配置参数重写 --------------------------
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
    
    # -------------------------- 5. 构建Launch描述 --------------------------
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # 添加点云坐标转换节点
    ld.add_action(Node(
        package='pointcloud_frame_trans',
        executable='pointcloud_frame_trans',
        name='pointcloud_frame_trans',
        output='screen',
        parameters=[
            configured_params,  # 从配置文件加载的参数
        ]
    ))
    
    # ld.add_action(Node(
    #     package='pointcloud_segmentation',
    #     executable='pointcloud_segmentation',
    #     name='pointcloud_segmentation',
    #     output='log',
    #     respawn=True,
    #     remappings=[
    #         ('pointcloud2_in', '/livox/pointcloud2/transframe'),
    #         ('pointcloud2_out', '/cloud_segmented')
    #     ],
    #     parameters=[
    #         {'leaf_size': 0.04},
    #         {'x_bound': 0.3},
    #         {'y_bound': 0.3},
    #         {'z_bound_high': 0.05},
    #         {'gradient_threshold': 1.0},
    #         {'pointcloud_output_frame_id': 'base_link'}
    #     ]
    # ))
    
    return ld
